#pragma once

#include <cstdint>
#include <cstring>
#include <stdexcept>
#include <iostream>

namespace GekkoDS {
    // fw declare types
    class MemStream;
    template <typename T> class Vec;
    template <typename Q, typename T> class SparseSet;

    // A simple dynamic array for trivially copyable types.
    template <typename T>
    class Vec {
        static_assert(std::is_trivially_copyable_v<T>, "DS::Vec<T> only supports trivially copyable types");
        static_assert(!std::is_pointer_v<T>, "DS::Vec<T> does not support pointer types");

        T* _data = nullptr;
        uint32_t _size = 0, _capacity = 0;

        void ensure_capacity(uint32_t min_capacity) {
            if (min_capacity <= _capacity) return;

            uint32_t new_capacity = _capacity ? _capacity : 8;
            while (new_capacity < min_capacity) {
                new_capacity *= 2;
            }

            T* new_data = new T[new_capacity];

            if (_data) {
                std::memcpy(new_data, _data, _size * sizeof(T));
                delete[] _data;
            }

            _data = new_data;
            _capacity = new_capacity;
        }

        template<typename U>
        friend void load_vec(Vec<U>& vec, MemStream& stream);

        template<typename U>
        friend void save_vec(Vec<U>& vec, MemStream& stream);
    public:
        ~Vec() {
            delete[] _data;
        }

        void push_back(const T& value) {
            ensure_capacity(_size + 1);
            _data[_size++] = value;
        }

        void push_back_range(const Vec<T>& other) {
            ensure_capacity(_size + other._size);
            std::memcpy(_data + _size, other._data, other._size * sizeof(T));
            _size += other._size;
        }

        void push_back_range(const T* values, uint32_t count) {
            ensure_capacity(_size + count);
            std::memcpy(_data + _size, values, count * sizeof(T));
            _size += count;
        }

        void pop_back() {
            if (_size > 0) {
                _size--;
            }
        }

        void remove_at(uint32_t index) {
            if (index >= _size) {
                return;
            }
            if (index != _size - 1) {
                std::swap(_data[index], _data[_size - 1]);
            }
            pop_back();
        }

        void remove_first(const T& value) {
            for (uint32_t i = 0; i < _size; ++i) {
                if (_data[i] == value) {
                    remove_at(i);
                    return;
                }
            }
        }

        bool contains(const T& value) const {
            for (uint32_t i = 0; i < _size; ++i) {
                if (_data[i] == value) {
                    return true;
                }
            }
            return false;
        }

        T& back() { return _data[_size - 1]; }
        const T& back() const { return _data[_size - 1]; }

        T& operator[](uint32_t i) { return _data[i]; }
        const T& operator[](uint32_t i) const { return _data[i]; }

        T& get(uint32_t index) { return _data[index]; }
        const T& get(uint32_t index) const { return _data[index]; }

        uint32_t size() const { return _size; }
        uint32_t capacity() const { return _capacity; }

        bool empty() const { return _size == 0; }

        void clear() { _size = 0; }

        T* data() { return _data; }

        T* begin() { return _data; }
        T* end() { return _data + _size; }

        const T* begin() const { return _data; }
        const T* end() const { return _data + _size; }
    };

    // Simple memory stream. Mostly used for saving and loading state of a component.
    class MemStream {
        bool _own_buffer;
        size_t _offset = 0;
        Vec<uint8_t>* _buffer;

    public:
        // gives you the option top pass your own buffer
        MemStream(Vec<uint8_t>* out_buffer = nullptr) {
            if (!out_buffer) {
                _buffer = new Vec<uint8_t>();
                _own_buffer = true;
            }
            else {
                _buffer = out_buffer;
                _own_buffer = false;
            }
        }

        ~MemStream() {
            if (_own_buffer) {
                delete _buffer;
            }
        }


        void write_chunk(const void* data, uint32_t size) {
            _buffer->push_back_range(reinterpret_cast<const uint8_t*>(&size), sizeof(size));
            _offset += sizeof(size);
            _buffer->push_back_range(reinterpret_cast<const uint8_t*>(data), size);
            _offset += size;
        }

        // Read size-prefixed chunk and advance offset past size+data automatically
        // Returns nullptr if not enough data
        const uint8_t* read_chunk(uint32_t& out_size) {
            if (_offset + sizeof(uint32_t) > _buffer->size())
                return nullptr;

            uint32_t size;
            std::memcpy(&size, _buffer->data() + _offset, sizeof(uint32_t));

            if (_offset + sizeof(uint32_t) + size > _buffer->size())
                return nullptr;

            _offset += sizeof(uint32_t); // move past size prefix

            const uint8_t* ptr = _buffer->data() + _offset;

            _offset += size; // move past the chunk data

            out_size = size;
            return ptr;
        }

        void rewind() { _offset = 0; }
        void seek(size_t offset) { _offset = (offset <= _buffer->size()) ? offset : _buffer->size(); }
        size_t tell() const { return _offset; }

        const uint8_t* data() { return _buffer->data(); }
        size_t size() const { return _buffer->size(); }
    };

    // SparseSet manages a collection of entities and their associated data.
    // It uses a signed integer type for IDs, with -1 representing an invalid ID.
    // Active entities are stored contiguously at the beginning of the dense vector.
    // Removed IDs are stored in free_ids for reuse.
    template <typename Q, typename T>
    class SparseSet {
        static_assert(std::is_integral_v<Q>, "DS::SparseSet<Q, T> requires Q to be an integral type");
        static_assert(std::is_signed_v<Q>, "DS::SparseSet<Q, T> requires Q to be signed because -1 is used as INVALID_ID");
        static_assert(std::is_trivially_copyable_v<T>, "DS::SparseSet<Q, T> requires T to be trivially copyable");

        Vec<T> _dense;       // Stores the actual data.

        Vec<Q> _sparse;      // Maps entity ID to its index in the dense vector.
        Vec<Q> _entities;    // Maps dense index back to its entity ID.
        Vec<Q> _free_ids;    // List of IDs available for reuse.

        Q _next_id = 0;      // Next new ID to assign.
        Q _active_count = 0; // Number of active (enabled) entities.

        // Swap the entries at two indices in the _dense vector (and update the mapping).
        void swap_dense(Q index1, Q index2) {
            std::swap(_dense[index1], _dense[index2]);
            std::swap(_entities[index1], _entities[index2]);
            _sparse[_entities[index1]] = index1;
            _sparse[_entities[index2]] = index2;
        }

    public:
        static constexpr Q INVALID_ID = -1;

        // Checks if the given id is valid (allocated and not removed).
        bool is_valid(Q id) const {
            if (id < 0) return false;
            uint32_t index = static_cast<uint32_t>(id);
            return index < _sparse.size() && _sparse[index] != INVALID_ID;
        }

        bool is_enabled(Q id) const {
            return is_valid(id) && (_sparse[id] < _active_count);
        }

        // Returns true if the set contains the given entity (i.e. if it is valid).
        bool contains(Q id) const {
            return is_valid(id);
        }

        // Inserts a new element; new elements start as active.
        Q insert(const T& value) {
            Q id;
            if (!_free_ids.empty()) {
                id = _free_ids.back();
                _free_ids.pop_back();
            }
            else {
                id = _next_id;
                if (_next_id == std::numeric_limits<Q>::max()) {
                    return INVALID_ID; // No more IDs available.
                }
                _next_id++;
            }

            // Ensure the sparse vector is large enough.
            while (static_cast<uint32_t>(id) >= _sparse.size()) {
                _sparse.push_back(INVALID_ID);
            }

            // Map the new entity to its dense index.
            _sparse[id] = _dense.size();
            _dense.push_back(value);
            _entities.push_back(id);

            // Swap the new element into the active region.
            if (_sparse[id] != _active_count) {
                swap_dense(_sparse[id], _active_count);
            }
            _active_count++;
            return id;
        }

        // Removes an entity from the set.
        void remove(Q id) {
            if (!is_valid(id)) {
                return;
            }

            Q index = _sparse[id];
            Q last_index = _dense.size() - 1;

            if (is_enabled(id)) {
                _active_count--;
            }

            if (index != last_index) {
                swap_dense(index, last_index);
            }

            _dense.pop_back();
            _entities.pop_back();

            _sparse[id] = INVALID_ID;

            _free_ids.push_back(id);
        }

        // Disables an active entity
        void disable(Q id) {
            if (is_valid(id) && is_enabled(id)) {
                Q index = _sparse[id];
                Q last_active = _active_count - 1;
                swap_dense(index, last_active);
                _active_count--;
            }
        }

        // Enables a disabled entity
        void enable(Q id) {
            if (is_valid(id) && !is_enabled(id)) {
                Q index = _sparse[id];
                swap_dense(index, _active_count);
                _active_count++;
            }
        }

        // Retrieves an entity by its ID.
        T& get(Q id) {
            if (!is_valid(id)) {
                throw std::out_of_range("Invalid ID");
            }
            return _dense[_sparse[id]];
        }

        const T& get(Q id) const {
            if (!is_valid(id)) {
                throw std::out_of_range("Invalid ID");
            }
            return _dense[_sparse[id]];
        }

        // Clears all entities.
        void clear() {
            _dense.clear();
            _entities.clear();
            _sparse.clear();
            _free_ids.clear();
            _next_id = 0;
            _active_count = 0;
        }

        void save(MemStream& stream) {
            stream.write_chunk(&_active_count, sizeof(Q));
            stream.write_chunk(&_next_id, sizeof(Q));
            save_vec(_free_ids,stream);
            save_vec(_sparse, stream);
            save_vec(_entities, stream);
            save_vec(_dense, stream);
        }

        void load(MemStream& stream) {
            uint32_t out_size = 0;
            // load _active_count
            auto data = stream.read_chunk(out_size);
            std::memcpy(&_active_count, data, out_size);
            // load _next_id
            data = stream.read_chunk(out_size);
            std::memcpy(&_next_id, data, out_size);
            // load the vecs
            load_vec(_free_ids, stream);
            load_vec(_sparse, stream);
            load_vec(_entities, stream);
            load_vec(_dense, stream);
        }

        void print_kv() const {
            for (uint32_t i = 0; i < size(); ++i) {
                Q id = _entities[i];
                const T& value = _dense[i];
                std::cout << "ID: " << id << ", Value: " << value << "\n";
            }
        }

        // Returns the total number of entities (active(enabled) + active(disabled)).
        uint32_t size() const { return static_cast<Q>(_dense.size()); }

        // Returns the number of active (enabled) entities.
        uint32_t active_size() const { return _active_count; }

        // Returns the number of active (disabled) entities.
        uint32_t disabled_size() const { return static_cast<Q>(_dense.size()) - _active_count; }

        // Iterators over active entities.
        T* begin() { return _dense.begin(); }
        T* end() { return _dense.begin() + _active_count; }

        const T* begin() const { return _dense.begin(); }
        const T* end() const { return _dense.begin() + _active_count; }

        const T* end_set() const { return _dense.begin() + size(); }
    };

    template <typename T>
    void save_vec(Vec<T>& vec, MemStream& stream) {
        stream.write_chunk(&vec._size, sizeof(uint32_t));
        stream.write_chunk(vec._data, vec._capacity * sizeof(T));
    }

    template <typename T>
    void load_vec(Vec<T>& vec, MemStream& stream) {
        uint32_t out_size = 0;
        // load _size
        auto data = stream.read_chunk(out_size);
        std::memcpy(&vec._size, data, out_size);
        // load _data and _capacity
        data = stream.read_chunk(out_size);
        vec.ensure_capacity(out_size / sizeof(T));
        std::memcpy(vec._data, data, out_size);
    }
} // namespace Gekko::DS