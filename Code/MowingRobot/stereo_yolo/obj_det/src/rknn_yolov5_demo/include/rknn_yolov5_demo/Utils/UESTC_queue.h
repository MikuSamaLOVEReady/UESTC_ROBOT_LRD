#include <mutex>
#include <deque>


namespace UESTC_STL{

    template <typename T>
    class mutex_queue{
        public:
            mutex_queue() = default;
        private:
            std::deque<T> dequeu_;
            std::mutex mutex_;

    };


}