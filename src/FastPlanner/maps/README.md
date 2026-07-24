# Maps

FastPlanner does not duplicate processed maps here. By default the launch file
loads these sibling MapProcessor results directly:

```text
../MapProcessor/maps/output/map_preprocessed.pcd
../MapProcessor/maps/output/map_preprocessed.bt
../MapProcessor/maps/tomogram/map_preprocessed.pickle
```

Set `MAP_PROCESSOR_ROOT` or pass `map_processor_root:=...` when the two projects
are not siblings.
