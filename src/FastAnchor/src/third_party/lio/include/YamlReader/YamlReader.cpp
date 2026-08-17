//
// Created by xiaofan on 24-11-25.
//

#include "YamlReader.h"
#include <utility>

// explicit instantiations
template bool YamlReader::read<std::string>(const std::string &, std::string &);
template bool YamlReader::read<int>(const std::string &, int &);
template bool YamlReader::read<double>(const std::string &, double &);
template bool YamlReader::read<float>(const std::string &, float &);
template bool YamlReader::read<bool>(const std::string &, bool &);
template bool YamlReader::readEigen<Eigen::Vector3d>(const std::string &, Eigen::MatrixBase<Eigen::Vector3d> &);
template bool YamlReader::readEigen<Eigen::Vector2d>(const std::string &, Eigen::MatrixBase<Eigen::Vector2d> &);
template bool YamlReader::readEigen<Eigen::Matrix3d>(const std::string &, Eigen::MatrixBase<Eigen::Matrix3d> &);
// You can continue to add more explicit instantiations as needed.


YamlReader::YamlReader(std::string filePath) : filePath_(std::move(filePath)) {}

bool YamlReader::load() {
    try {
        rootNode_ = std::make_shared<YAML::Node>(YAML::LoadFile(filePath_));
        return true;
    } catch (const YAML::BadFile &e) {
        std::cerr << "Failed to load YAML file: " << e.what() << std::endl;
        return false;
    }
}

template <typename T>
bool YamlReader::read(const std::string &key, T &value) {
    YAML::Node node;
    if (!getValueByKey(key, node)) {
        std::cerr << "Failed to find key: " << key << std::endl;
        return false;
    }

    try {
        value = node.as<T>();
        return true;
    } catch (const YAML::BadConversion &e) {
        std::cerr << "Type conversion error for key '" << key << "': " << e.what() << std::endl;
        return false;
    }
}

template <typename Derived>
bool YamlReader::readEigen(const std::string &key, Eigen::MatrixBase<Derived> &matrix) {
    YAML::Node node;
    if (!getValueByKey(key, node)) {
        std::cerr << "Failed to find key: " << key << std::endl;
        return false;
    }

    if constexpr (Derived::ColsAtCompileTime == 1 || Derived::RowsAtCompileTime == 1) {
        // process vector
        if (!node.IsSequence() || node.size() != matrix.size()) {
            std::cerr << "Node is not a sequence or size mismatch for key: " << key << std::endl;
            return false;
        }

        for (std::size_t i = 0; i < node.size(); ++i) {
            matrix(i) = node[i].as<typename Derived::Scalar>();
        }
    } else {
        // process matrix
        if (!node.IsSequence() || node.size() != matrix.rows()) {
            std::cerr << "Node is not a sequence or row size mismatch for key: " << key << std::endl;
            return false;
        }

        for (std::size_t i = 0; i < node.size(); ++i) {
            if (!node[i].IsSequence() || node[i].size() != matrix.cols()) {
                std::cerr << "Column size mismatch at row " << i << " for key: " << key << std::endl;
                return false;
            }

            for (std::size_t j = 0; j < node[i].size(); ++j) {
                matrix(i, j) = node[i][j].as<typename Derived::Scalar>();
            }
        }
    }

    return true;
}

template <typename T>
bool YamlReader::getValueByKey(const std::string &key, T &value) {
    load(); // mind that you must reload rootNode_ because rootNode_ will be changed by itself when you traverse it
    YAML::Node currentNode = *rootNode_;
    // std::cout << currentNode << std::endl;
    // std::cout <<"AAA" << std::endl;
    std::istringstream keyStream(key);
    std::string subKey;

    while (std::getline(keyStream, subKey, delimiter_)) {
        if (currentNode[subKey]) {
            currentNode = currentNode[subKey];
        } else {
            return false;
        }
    }
    value = currentNode;
    return true;
}
