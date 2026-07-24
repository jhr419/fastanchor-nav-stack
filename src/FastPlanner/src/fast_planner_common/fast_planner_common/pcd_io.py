import os
from pathlib import Path
import struct

import numpy as np


def _log_warning(logger, message):
    if logger is None:
        return
    if hasattr(logger, 'warning'):
        logger.warning(message)
    else:
        logger.warn(message)


def _pcd_dtype(type_name, size):
    if type_name == 'F':
        if size == 4:
            return '<f4'
        if size == 8:
            return '<f8'
    if type_name == 'I':
        if size in (1, 2, 4, 8):
            return '<i%d' % size
    if type_name == 'U':
        if size in (1, 2, 4, 8):
            return '<u%d' % size

    raise ValueError('Unsupported PCD field type: %s%d' % (type_name, size))


def _read_pcd_header(pcd_file):
    header = {}
    while True:
        line = pcd_file.readline()
        if not line:
            raise ValueError('Invalid PCD file: missing DATA header')

        line = line.decode('utf-8', errors='replace').strip()
        if not line or line.startswith('#'):
            continue

        tokens = line.split()
        key = tokens[0].upper()
        header[key] = tokens[1:]
        if key == 'DATA':
            if len(tokens) < 2:
                raise ValueError('Invalid PCD file: DATA header is empty')
            return header


def _pcd_points_count(header):
    if 'POINTS' in header:
        return int(header['POINTS'][0])
    if 'WIDTH' in header and 'HEIGHT' in header:
        return int(header['WIDTH'][0]) * int(header['HEIGHT'][0])
    raise ValueError('Invalid PCD file: missing POINTS or WIDTH/HEIGHT')


def _pcd_xyz_columns(fields, counts):
    offsets = {}
    column = 0
    for field, count in zip(fields, counts):
        offsets.setdefault(field, column)
        column += count

    try:
        return [offsets['x'], offsets['y'], offsets['z']]
    except KeyError as exc:
        raise ValueError('PCD file must contain x, y, and z fields') from exc


def _lzf_decompress(data, expected_size):
    output = bytearray()
    index = 0
    while index < len(data):
        control = data[index]
        index += 1
        if control < 32:
            length = control + 1
            output.extend(data[index:index + length])
            index += length
            continue

        length = control >> 5
        reference = len(output) - ((control & 0x1F) << 8) - 1
        if length == 7:
            length += data[index]
            index += 1
        reference -= data[index]
        index += 1
        length += 2
        if reference < 0:
            raise ValueError('Invalid LZF back reference in binary_compressed PCD')
        for _ in range(length):
            output.append(output[reference])
            reference += 1
    if len(output) != expected_size:
        raise ValueError(
            'Invalid LZF output size: expected %d, got %d' % (expected_size, len(output))
        )
    return bytes(output)


def _read_binary_compressed_xyz(pcd_file, fields, sizes, types, counts, point_count):
    sizes_header = pcd_file.read(8)
    if len(sizes_header) != 8:
        raise ValueError('Invalid binary_compressed PCD size header')
    compressed_size, uncompressed_size = struct.unpack('<II', sizes_header)
    compressed = pcd_file.read(compressed_size)
    if len(compressed) != compressed_size:
        raise ValueError('Truncated binary_compressed PCD payload')
    raw = _lzf_decompress(compressed, uncompressed_size)

    field_offsets = {}
    offset = 0
    for field, size, type_name, count in zip(fields, sizes, types, counts):
        field_offsets[field] = (offset, _pcd_dtype(type_name, size), count)
        offset += size * count * point_count
    xyz = []
    for field in ('x', 'y', 'z'):
        if field not in field_offsets:
            raise ValueError('PCD file must contain x, y, and z fields')
        offset, dtype, count = field_offsets[field]
        values = np.frombuffer(raw, dtype=dtype, count=point_count * count, offset=offset)
        xyz.append(values.reshape(point_count, count)[:, 0])
    return np.column_stack(xyz).astype(np.float32, copy=False)


def _read_pcd_xyz_fallback(pcd_path):
    with open(pcd_path, 'rb') as pcd_file:
        header = _read_pcd_header(pcd_file)

        fields = header.get('FIELDS')
        sizes = [int(value) for value in header.get('SIZE', [])]
        types = header.get('TYPE')
        counts = [int(value) for value in header.get('COUNT', [])]

        if not fields or not sizes or not types:
            raise ValueError('Invalid PCD file: missing FIELDS, SIZE, or TYPE')
        if not counts:
            counts = [1] * len(fields)
        if not (len(fields) == len(sizes) == len(types) == len(counts)):
            raise ValueError('Invalid PCD file: inconsistent field metadata')

        data_kind = header['DATA'][0].lower()
        point_count = _pcd_points_count(header)

        if data_kind == 'ascii':
            columns = _pcd_xyz_columns(fields, counts)
            points = np.loadtxt(pcd_file, dtype=np.float32, usecols=columns)
            return np.atleast_2d(points).astype(np.float32, copy=False)

        if data_kind == 'binary_compressed':
            return _read_binary_compressed_xyz(
                pcd_file, fields, sizes, types, counts, point_count
            )

        if data_kind != 'binary':
            raise NotImplementedError('Unsupported PCD DATA encoding: %s' % data_kind)

        dtype_fields = []
        dtype_names = {}
        for index, (field, size, type_name, count) in enumerate(zip(fields, sizes, types, counts)):
            dtype_name = field if field not in dtype_names.values() else '%s_%d' % (field, index)
            dtype_names.setdefault(field, dtype_name)
            dtype = _pcd_dtype(type_name, size)
            if count == 1:
                dtype_fields.append((dtype_name, dtype))
            else:
                dtype_fields.append((dtype_name, dtype, (count,)))

        point_dtype = np.dtype(dtype_fields)
        data = np.fromfile(pcd_file, dtype=point_dtype, count=point_count)
        if data.shape[0] != point_count:
            raise ValueError('Invalid PCD file: expected %d points, read %d' % (point_count, data.shape[0]))

        xyz = []
        for field in ('x', 'y', 'z'):
            if field not in dtype_names:
                raise ValueError('PCD file must contain x, y, and z fields')
            values = data[dtype_names[field]]
            if values.ndim > 1:
                values = values[:, 0]
            xyz.append(values)

        return np.column_stack(xyz).astype(np.float32, copy=False)


def read_pcd_xyz(pcd_path, logger=None):
    if not os.path.isfile(pcd_path):
        raise FileNotFoundError('PCD file not found: %s' % pcd_path)

    return _read_pcd_xyz_fallback(pcd_path)


def _write_pcd_xyz_fallback(pcd_path, points):
    points = np.asarray(points, dtype=np.float32)
    Path(pcd_path).parent.mkdir(parents=True, exist_ok=True)
    header = (
        '# .PCD v0.7 - Point Cloud Data file format\n'
        'VERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n'
        'WIDTH %d\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS %d\nDATA binary\n'
        % (points.shape[0], points.shape[0])
    ).encode('ascii')
    with open(pcd_path, 'wb') as pcd_file:
        pcd_file.write(header)
        np.ascontiguousarray(points[:, :3], dtype='<f4').tofile(pcd_file)


def write_pcd_xyz(pcd_path, points, logger=None):
    Path(pcd_path).parent.mkdir(parents=True, exist_ok=True)
    _write_pcd_xyz_fallback(pcd_path, points)
