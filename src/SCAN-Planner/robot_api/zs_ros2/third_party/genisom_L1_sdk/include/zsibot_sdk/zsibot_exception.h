#ifndef ZSIBOT_EXCEPTION_H
#define ZSIBOT_EXCEPTION_H

#include <exception>
#include <string>

namespace zsibot
{

class ZsibotException : public std::exception
{
   public:
    explicit ZsibotException(const std::string &message) : m_message(message) {}
    ~ZsibotException() = default;

    const char *what() const noexcept override
    {
        return m_message.c_str();  // 返回异常信息的 C 字符串
    }

   private:
    std::string m_message;
};
}  // namespace zsibot

#endif  // ZSIBOT_EXCEPTION_H