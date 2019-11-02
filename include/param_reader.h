#ifndef _PARAM_READER_H_
#define _PARAM_READER_H_
#include <memory>
#include <vector>
#include <string>
#include <sstream>
#include <iostream>

/**
 * copy from tergeo commom, for not depend on tergeo
 * 
*/
class ParamReaderPrivate;
/// @brief 参数文件读取
class ParamReader {
public:
    typedef std::shared_ptr<ParamReader> Ptr;
    ParamReader();
    ParamReader(const std::string file_path, bool case_sensitive = false);
    ~ParamReader();
    /**
     * @brief 索引值是否大小写敏感
     * @param ture: 区分大小写
     *        false: 不区分大小写，Key、KEY等同 
     */ 
    void setKeyCaseSensitive(bool case_sensitive);
    /**
     * @brief 加载参数文件
     * @param input 参数文件路径名
     * @param input 是否采用追加方式，
     *              ture: 不清空当前已经读取的值，适用于从多文件读取参数
     *              false: 清空当已经读取的值，单文件读取
     * @return 返回是否读取成功
     */ 
    bool loadParam(const std::string& file_path, bool append = false);
    bool saveParam(const std::string& file_path);
    bool has(const std::string& key) const;

    template <class T>
    bool getValue(const std::string& key, T& value) {
        if (!has(key)) {
            return false;
        }
        std::stringstream ss(getStringValue(key));
        ss >> value;
        return true;
    }
    template <class T>
    bool getValueVec(const std::string& key, std::vector<T>& values) {
        if (!has(key)) {
            return false;
        }
        values.clear();
        std::stringstream ss(getStringValue(key));
        T value;
        while (ss >> value) {
            values.push_back(value);
        }
        return true;
    }
    template <class T>
    bool setValue(std::string key, T value) {
        std::stringstream ss;
        ss << value;
        return setStringValue(key, ss.str());
    }
    template <class T>
    bool setValueVec(std::string key, std::vector<T>& values) {
        std::stringstream ss;
        if (values.empty()) {
            std::cerr << "Values vec empty!";
            return false;
        }
        ss << values[0];
        for (int i = 1; i < values.size(); ++i) {
            ss << " " << values[i];
        }
        return setStringValue(key, ss.str());
    }

private:
    std::string getStringValue(const std::string& key) const;
    bool setStringValue(std::string key, std::string value);

private:
    ParamReaderPrivate* _ptr;
};

#endif