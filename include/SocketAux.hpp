#pragma once
#include <asio.hpp>
#include <iostream>
#include <optional>

namespace RGBD {
	class Server {
	public:
		Server();
		~Server();

		bool send(std::string data);
		bool send(const char* data, int32_t dataLen);

		// send int as byte array
		bool send(int32_t data);

		std::optional<std::string> rcv();

	private:
		const int PORT = 8888;

		asio::ip::tcp::socket* socket_;
		asio::io_context* io_context_;

		std::optional<char*> readSocket(size_t size);
		std::optional<int> getIntFromSocket();
		constexpr int bytesToInt(const char* bytes);
		char* intToByteArray(int value);
	};
}