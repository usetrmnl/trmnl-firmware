#include <http_client.h>
#include <http_utils.h>
#include <open_connections.h>

namespace {
  struct Connection {
    String origin;
    WiFiClient *client = nullptr;
    HTTPClient *http = nullptr;
  };

  bool kept = false;
  Connection connections[2]; // the API and the image host
  uint8_t next_to_replace = 0;

  void close(Connection &connection) {
    delete connection.http; // its destructor stops the client
    delete connection.client;
    connection = Connection();
  }

  Connection &connectionFor(const String &origin) {
    for (Connection &connection : connections)
      if (connection.origin == origin) return connection;
    Connection &connection = connections[next_to_replace];
    next_to_replace ^= 1;
    close(connection);
    connection.origin = origin;
    connection.client = newWiFiClientFor(origin, /*resumable=*/true);
    connection.http = new HTTPClient();
    return connection;
  }
} // namespace

void OpenConnections::keep(bool on) {
  kept = on;
  if (!on) closeAll();
}

void OpenConnections::closeAll() {
  for (Connection &connection : connections)
    close(connection);
}

HTTPClient *OpenConnections::begin(const String &url) {
  if (!kept) return nullptr;
  Connection &connection = connectionFor(httpOriginOf(url));
  if (!connection.http->begin(*connection.client, url)) return nullptr;
  connection.http->setReuse(true);
  return connection.http;
}

void OpenConnections::end(HTTPClient *http, bool request_succeeded) {
  http->end(); // keeps the connection open when the server allows it
  for (Connection &connection : connections)
    if (connection.http == http && (!request_succeeded || !connection.client->connected())) connection.client->stop();
}
