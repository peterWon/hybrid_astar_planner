#include <map>
#include <fstream>
#include <iostream>
#include <algorithm>
#include "param_reader.h"


class ParamReaderPrivate {
public:
    ParamReaderPrivate(ParamReader* pthis) : _pthis(pthis), _case_sensitive(false) {}
    ~ParamReaderPrivate() {}
    std::string toUpper(const std::string& str) {
        std::string tmp = str;
        std::transform(tmp.begin(), tmp.end(), tmp.begin(), toupper);
        return std::move(tmp);
    }
    std::string trim(std::string str) {
        std::size_t pos_1 = str.find_first_not_of(' ');
        std::size_t pos_2 = str.find_last_not_of(' ');
        return str.substr(pos_1, pos_2 - pos_1 + 1);
    }
    std::string trimQuatation(std::string str) {
        std::size_t pos_1 = str.find_first_not_of('"');
        std::size_t pos_2 = str.find_last_not_of('"');
        return str.substr(pos_1, pos_2 - pos_1 + 1);
    }
    std::string preProcessKey(std::string key) {
        key = trim(key);
        key = trimQuatation(key);
        if (!_case_sensitive) {
            return toUpper(key);
        }
        return key;
    }
    std::string preProcessValue(std::string value) {
        std::size_t pos_1 = value.find_first_of('[');
        std::size_t pos_2 = value.find_last_of(']');
        if (pos_1 != std::string::npos && pos_2 != std::string::npos) {
            value = value.substr(pos_1 + 1, pos_2 - pos_1 - 1);
        } else if(pos_1 == std::string::npos && pos_2 == std::string::npos) {
            // do nothing
        } else {
            std::cerr << "Param file syntext error!"<<std::endl;
        }
        while (value.find_first_of(',') != std::string::npos) {
            value = value.replace(value.find(','), 1, 1, ' ');
        }
        std::string trimmed_value = trim(value);
        return trimQuatation(trimmed_value);
    }
    bool loadParam(const std::string& param_file, bool appand) {
        if (!appand) {_param_map.clear();}
        std::ifstream in_file(param_file);
        if (!in_file.is_open()) {
            std::cerr << "Can not open param file: " << param_file<<std::endl;
            return false;
        }
        while (!in_file.eof()) {
            std::string line_str;
            std::getline(in_file, line_str);
            if (line_str.empty()) {
                continue;
            }
            std::size_t pos = line_str.find(':');
            if (pos == std::string::npos) {
                continue;
            }
            std::string key = preProcessKey(line_str.substr(0, pos));
            std::string value = preProcessValue(line_str.substr(pos + 1));
            if (key.find_first_of('#') != std::string::npos) {
                continue;
            }
            std::cout << "Param: " << key << " " << value<<std::endl;
            _param_map[key] = value;
            if (!in_file.good()) {
                break;
            }
        }
        in_file.close();
        return true;
    }
    bool saveParam(std::string path) {
        std::ofstream out_file(path, std::ios::out);
        if (!out_file.is_open()) {
            std::cerr << "Can not save param: " << path<<std::endl;
            return false;
        }
        std::map<std::string, std::string>::const_iterator iter;
        for (iter = _param_map.begin(); iter != _param_map.end(); ++iter) {
            out_file << iter->first << ": " << iter->second << std::endl;
        }
        out_file.close();
        return true;
    }
    bool has(const std::string& key) {
        bool flag = _param_map.find(preProcessKey(key)) != _param_map.end();
        if (!flag) {
            std::cerr << "Do not have param key: " << key<<std::endl;
        }
        return flag;
    }
    std::string getParamString(const std::string& key) {
        std::map<std::string, std::string>::const_iterator iter
            = _param_map.find(preProcessKey(key));
        if (iter == _param_map.end()) {
            return "";
        }
        return iter->second;
    }
    bool setStringValue(std::string key, std::string value) {
        if (key.empty() || key.empty()) {
            return false;
        }
        if (!_case_sensitive) {
            key = toUpper(key);
        }
        _param_map[key] = value;
        return true;
    }

private:
    std::map<std::string, std::string> _param_map;
    bool _case_sensitive;

    ParamReader* _pthis;
    friend class ParamReader;
};
ParamReader::ParamReader() {
    _ptr = new ParamReaderPrivate(this);
    _ptr->_case_sensitive = false;
}
ParamReader::ParamReader(const std::string file_path, bool case_sensitive) {
    _ptr = new ParamReaderPrivate(this);
    _ptr->_case_sensitive = case_sensitive;
    if (!file_path.empty()) {
        _ptr->loadParam(file_path, false);
    }
}
ParamReader::~ParamReader() {
    if (_ptr) {
        delete _ptr;
        _ptr = nullptr;
    }
}
void ParamReader::setKeyCaseSensitive(bool case_sensitive) {
    _ptr->_case_sensitive = case_sensitive;
}
bool ParamReader::loadParam(const std::string& file_path, bool append) {
    return _ptr->loadParam(file_path, append);
}
bool ParamReader::saveParam(const std::string& file_path) {
    return _ptr->saveParam(file_path);
}
bool ParamReader::has(const std::string& key) const {
    return _ptr->has(key);
}
std::string ParamReader::getStringValue(const std::string& key) const {
    return _ptr->getParamString(key);
}
bool ParamReader::setStringValue(std::string key, std::string value) {
    return _ptr->setStringValue(key, value);
}