#pragma once

#include <type_traits>
#include <utility>

template <typename F>
class DeferImpl
{
public:
    explicit DeferImpl(F && f) : m_fn(std::forward<F>(f)), m_active(true) {}
    DeferImpl(const DeferImpl &) = delete;
    DeferImpl & operator=(const DeferImpl &) = delete;
    DeferImpl(DeferImpl && other) noexcept
        : m_fn(std::move(other.m_fn)), m_active(other.m_active)
    {
        other.cancel();
    }
    DeferImpl & operator=(DeferImpl &&) = delete;

    ~DeferImpl()
    {
        if (m_active)
            m_fn();
    }

    void cancel() { m_active = false; }

private:
    F    m_fn;
    bool m_active;
};

template <typename F>
[[nodiscard]] auto makeDefer(F && f)
{
    return DeferImpl<std::decay_t<F>>(std::forward<F>(f));
}

#define DEFER_CONCAT2(a, b) a##b
#define DEFER_CONCAT1(a, b) DEFER_CONCAT2(a, b)
#define DEFER(code) \
    auto DEFER_CONCAT1(_defer_, __COUNTER__) = makeDefer([&]() { code; });
