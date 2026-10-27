#pragma once

#include <array>
#include <cstddef>
#include <iterator>
#include <stdexcept>

/**
 * @brief Zero-allocation contiguous circular buffer with fixed compile-time capacity.
 * 
 * Designed for high-frequency telemetry and real-time physical simulation loops (60 Hz+).
 * Eliminates heap allocations (malloc/realloc) inside OpenMP hot-paths and maintains
 * deterministic cache locality.
 * 
 * @tparam T Type of stored elements.
 * @tparam Capacity Maximum number of elements statically preallocated.
 */
template <typename T, std::size_t Capacity>
class FixedRingBuffer {
    static_assert(Capacity > 0, "FixedRingBuffer Capacity must be greater than 0");

private:
    alignas(alignof(T)) std::array<T, Capacity> m_data{};
    std::size_t m_head = 0; // Index of oldest element
    std::size_t m_size = 0; // Number of active elements

public:
    constexpr FixedRingBuffer() = default;

    /**
     * @brief Optimized copy constructor that only copies active elements.
     */
    FixedRingBuffer(const FixedRingBuffer& other) {
        copy_from(other);
    }

    /**
     * @brief Optimized copy assignment operator that only copies active elements.
     */
    FixedRingBuffer& operator=(const FixedRingBuffer& other) {
        if (this != &other) {
            copy_from(other);
        }
        return *this;
    }

    FixedRingBuffer(FixedRingBuffer&&) noexcept = default;
    FixedRingBuffer& operator=(FixedRingBuffer&&) noexcept = default;
    ~FixedRingBuffer() = default;

    /**
     * @brief Pushes an item into the ring buffer without dynamic memory allocation.
     * Overwrites the oldest item if capacity is reached.
     */
    void push_back(const T& item) {
        if (m_size < Capacity) {
            std::size_t write_idx = (m_head + m_size) % Capacity;
            m_data[write_idx] = item;
            m_size++;
        } else {
            m_data[m_head] = item;
            m_head = (m_head + 1) % Capacity;
        }
    }

    void push_back(T&& item) {
        if (m_size < Capacity) {
            std::size_t write_idx = (m_head + m_size) % Capacity;
            m_data[write_idx] = std::move(item);
            m_size++;
        } else {
            m_data[m_head] = std::move(item);
            m_head = (m_head + 1) % Capacity;
        }
    }

    /**
     * @brief Resets buffer size to 0 in O(1) time without deallocation.
     */
    void clear() noexcept {
        m_head = 0;
        m_size = 0;
    }

    [[nodiscard]] constexpr std::size_t size() const noexcept { return m_size; }
    [[nodiscard]] constexpr bool empty() const noexcept { return m_size == 0; }
    [[nodiscard]] constexpr std::size_t capacity() const noexcept { return Capacity; }

    /**
     * @brief Subscript access by logical index (0 to size() - 1).
     */
    const T& operator[](std::size_t index) const noexcept {
        return m_data[(m_head + index) % Capacity];
    }

    T& operator[](std::size_t index) noexcept {
        return m_data[(m_head + index) % Capacity];
    }

    const T& at(std::size_t index) const {
        if (index >= m_size) {
            throw std::out_of_range("FixedRingBuffer index out of range");
        }
        return (*this)[index];
    }

    T& at(std::size_t index) {
        if (index >= m_size) {
            throw std::out_of_range("FixedRingBuffer index out of range");
        }
        return (*this)[index];
    }

    const T& front() const noexcept { return (*this)[0]; }
    T& front() noexcept { return (*this)[0]; }

    const T& back() const noexcept { return (*this)[m_size - 1]; }
    T& back() noexcept { return (*this)[m_size - 1]; }

    // Random Access Iterator
    class const_iterator {
    private:
        const FixedRingBuffer* m_buffer = nullptr;
        std::size_t m_index = 0;

    public:
        using iterator_category = std::random_access_iterator_tag;
        using value_type = T;
        using difference_type = std::ptrdiff_t;
        using pointer = const T*;
        using reference = const T&;

        constexpr const_iterator() = default;
        constexpr const_iterator(const FixedRingBuffer* buf, std::size_t idx) : m_buffer(buf), m_index(idx) {}

        reference operator*() const noexcept { return (*m_buffer)[m_index]; }
        pointer operator->() const noexcept { return &(*m_buffer)[m_index]; }

        const_iterator& operator++() noexcept { ++m_index; return *this; }
        const_iterator operator++(int) noexcept { const_iterator tmp = *this; ++m_index; return tmp; }
        const_iterator& operator--() noexcept { --m_index; return *this; }
        const_iterator operator--(int) noexcept { const_iterator tmp = *this; --m_index; return tmp; }

        const_iterator& operator+=(difference_type n) noexcept { m_index += n; return *this; }
        const_iterator& operator-=(difference_type n) noexcept { m_index -= n; return *this; }
        const_iterator operator+(difference_type n) const noexcept { return const_iterator(m_buffer, m_index + n); }
        const_iterator operator-(difference_type n) const noexcept { return const_iterator(m_buffer, m_index - n); }
        difference_type operator-(const const_iterator& other) const noexcept { return static_cast<difference_type>(m_index) - static_cast<difference_type>(other.m_index); }

        reference operator[](difference_type n) const noexcept { return (*m_buffer)[m_index + n]; }

        bool operator==(const const_iterator& other) const noexcept { return m_buffer == other.m_buffer && m_index == other.m_index; }
        bool operator!=(const const_iterator& other) const noexcept { return !(*this == other); }
        bool operator<(const const_iterator& other) const noexcept { return m_index < other.m_index; }
        bool operator>(const const_iterator& other) const noexcept { return m_index > other.m_index; }
        bool operator<=(const const_iterator& other) const noexcept { return m_index <= other.m_index; }
        bool operator>=(const const_iterator& other) const noexcept { return m_index >= other.m_index; }
    };

    const_iterator begin() const noexcept { return const_iterator(this, 0); }
    const_iterator end() const noexcept { return const_iterator(this, m_size); }
    const_iterator cbegin() const noexcept { return const_iterator(this, 0); }
    const_iterator cend() const noexcept { return const_iterator(this, m_size); }

private:
    void copy_from(const FixedRingBuffer& other) {
        m_size = other.m_size;
        m_head = 0; // Flatten memory layout for maximized linear cache locality
        for (std::size_t i = 0; i < m_size; ++i) {
            m_data[i] = other[i];
        }
    }
};
