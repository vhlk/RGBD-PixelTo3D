#include <SocketAux.hpp>

RGBD::Server::Server()
{
	using namespace asio;
	using namespace ip;

	io_context_ = new io_context();

	// listen
	tcp::acceptor acceptor_(*io_context_, tcp::endpoint(tcp::v4(), PORT), true);

	// create socket
	socket_ = new tcp::socket(*io_context_);

	// wait cliente
	acceptor_.accept(*socket_);
}

RGBD::Server::~Server() {
	io_context_->stop();
	socket_->close();
	delete io_context_;
}

bool RGBD::Server::send(std::string data)
{
	using namespace asio;
	// first we send the data lenght
	if (!send(data.length()))
		return false;

	asio::error_code ec;
	write(*socket_, buffer(data), ec);

	if (ec) [[unlikely]]
		return false;

	return true;
}

bool RGBD::Server::send(const char* data, int32_t dataLen)
{
	using namespace asio;
	// first we send the data lenght
	if (!send(dataLen))
		return false;

	asio::error_code ec;
	write(*socket_, buffer(data, dataLen), ec);

	if (ec) [[unlikely]]
		return false;

	return true;
}

bool RGBD::Server::send(int32_t data)
{
	using namespace asio;
	asio::error_code ec;

	write(*socket_, buffer(intToByteArray(data), 4), ec);

	if (ec) [[unlikely]] {
		std::cout << ec.message() << '\n';
		return false;
	}

	return true;
}

std::optional<std::string> RGBD::Server::rcv()
{
	// we first get the string size (and if has landmarks)
	auto optHasLandmarkAndSize = getIntFromSocket();

	if (!optHasLandmarkAndSize.has_value()) [[unlikely]]
		return std::nullopt;

	auto hasLandMarksAndSize = optHasLandmarkAndSize.value();

	if (hasLandMarksAndSize == -2) // not landmarks found
		return std::nullopt;

	auto landmarks = readSocket(hasLandMarksAndSize);

	if (!landmarks.has_value()) [[unlikely]]
		return std::nullopt;

	return std::string(landmarks.value(), hasLandMarksAndSize);
}

std::optional<char*> RGBD::Server::readSocket(size_t size)
{
	char* output = new char[size];
	asio::error_code ec;

	std::size_t bytesReceived = asio::read(*socket_, asio::buffer(output, size), asio::transfer_exactly(size), ec);

	if (ec) [[unlikely]]
		return std::nullopt;

	return output;
}

std::optional<int> RGBD::Server::getIntFromSocket()
{
	const auto intChars = readSocket(4);

	if (!intChars.has_value()) [[unlikely]]
		return std::nullopt;

	int size = bytesToInt(intChars.value());

	return size;
}

constexpr int RGBD::Server::bytesToInt(const char* bytes)
{
	return (int)((unsigned char)bytes[0] << 24 |
		(unsigned char)bytes[1] << 16 |
		(unsigned char)bytes[2] << 8 |
		(unsigned char)bytes[3]);
}

char* RGBD::Server::intToByteArray(int32_t value)
{
	using namespace std;

	char* b = new char[4];
	b[0] = (value >> 24 & 0xFF);
	b[1] = (value >> 16 & 0xFF);
	b[2] = (value >> 8 & 0xFF);
	b[3] = (value & 0xFF);

	return b;
}
