# UWRT_UTILS PACKAGE

This package contains some useful libraries that are commonly used throughout `uwrt_mars_rover`. Each "library" should ideally consists of one `.h` and `.cpp` file, along with a node to test it with (which comes with its own `.cpp` and `.launch` file).

## CAN Libary

This is the main way to interact with the CAN bus. It's essentially an socket can wrapper with a rx thread that constantly looks for new messages from the desired ids, populating a `map<id, data>` with the new data, and overwriting the older data. When a user reads from an id that message is cleared from the map, so there won't be any stale readings either. Interfacing with this wrapper works as follows:

`UWRTCANWrapper` - class for interacting with the CAN bus. Initialize it with:
- `std::string name` - a name for the wrapper, should be the same as the node/plugin it's being used in
- `std::string interface_name` - name of the CAN interface (normally can0)
- `bool rcv_big_endian` - whether the device you are communicating with sends the data in big endian or not
- `int thread_sleep_millis = 10` - (optional) the speed at which to run the rx thread

`void init()` - initializes the can socket and starts the rx thread
- `std::vector<uint32_t> ids` - list of ids for the rx thread to pay attention to

`bool getLatestFromID<T>(T& data, uint32_t id)` - checks if there is a new message from the id
- `T data` - places new data from the id if the wrapper has any, `<T>` can be any primative type (as long as its not too big)
- `uint32_t id` the id to get new data from. __This id must be included in the `vector` you used to `init` the wrapper with__
- `returns`: `true` if there was a new message, `false` if not

`bool writeToID<T>(T data, uint32_t id)` - writes data to a specified id
- `T data` - data to send over can, `<T>` can be any primative type (as long as its not too big)
- `uint32_t id` the id to send the data to
- `returns`: `true` if the send was successfull, `false` if not

For an example on how to use the wrapper, look at `uwrt_utils_can_test_node.cpp`
