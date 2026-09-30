# Hello world example

This example demonstrates the basic usage of the CreOS SDK. The code is written in C++ and uses the CreOS SDK Client to interact with the SDK server. View the [main.cpp](src/main.cpp) file to see how to use the SDK to make an API call.

## Usage

```bash
./build/bin/hello_world_example
```

## Expected output

The example subscribes to battery status messages and prints the battery state of charge to the console whenever a new message is received:

```
Battery state of charge: 85%
Battery state of charge: 85%
...
```
