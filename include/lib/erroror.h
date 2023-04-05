#pragma once

#include <cstdarg>
#include <cstdio>
#include <cstring>
#include <utility>
#include <cstdarg>
#include <optional>

/**
 * @brief ErrorOr return type for signaling error conditions
 */
class Error
{
public:
  explicit Error(const char *msg)
  {
    if (strlen(msg) > 128 - 1)
    {
      // fixme: PANIC
      strcpy(m_msg, "###: MSG TOO LONG");
    }
    else
    {
      strcpy(m_msg, msg);
    }
  }

  static Error errorf(const char *fmt, ...)
  {
    Error err;
    va_list args;
    va_start(args, fmt);
    vsnprintf(err.m_msg, 128, fmt, args);
    va_end(args);
    return err;
  }

  Error(const Error &rhs) { strcpy(m_msg, rhs.m_msg); }

  Error(Error &&rhs) { strcpy(m_msg, rhs.m_msg); }

  explicit Error()
  {
    m_msg[0] = 0;
    valid = false;
  }

  const char *message() { return m_msg; }

  bool is_valid() const { return valid; }

private:
  char m_msg[128];
  bool valid = true;
};

template <typename T>
class [[nodiscard]] ErrorOr
{
public:
  ErrorOr(T const &value)
      : m_value(value)
  {
  }

  ErrorOr(T &&value)
      : m_value(std::move(value))
  {
  }

  ErrorOr(Error &&error)
      : m_error(std::move(error))
  {
  }

  ErrorOr(ErrorOr &&other) = default;
  ErrorOr(ErrorOr const &other) = default;
  ~ErrorOr() = default;

  T &value() { return m_value.value(); }
  Error &error() { return m_error.value(); }

  bool is_error() const { return m_error.has_value(); }

  T release_value() { return m_value.release_value(); }
  Error release_error() { return m_error.release_value(); }

private:
  std::optional<T> m_value;
  std::optional<Error> m_error;
};

template <>
class [[nodiscard]] ErrorOr<void>
{
public:
  ErrorOr(Error error)
      : m_error(move(error))
  {
  }

  ErrorOr() = default;
  ErrorOr(ErrorOr &&other) = default;
  ErrorOr(const ErrorOr &other) = default;
  ~ErrorOr() = default;

  Error &error() { return m_error.value(); }
  bool is_error() const { return m_error.has_value(); }
  Error release_error() { return m_error.release_value(); }

private:
  optional<Error> m_error;
};

#define TRY(expr)                       \
  ({                                    \
    auto result = (expr);               \
    if (result.is_error()) [[unlikely]] \
      return result.release_error();    \
    result.release();                   \
  })
