
#ifndef GUARD_MLOPEN_RETURNS_HPP
#define GUARD_MLOPEN_RETURNS_HPP

#define MLOPEN_RETURNS(...) -> decltype(__VA_ARGS__) { return __VA_ARGS__; }

#endif