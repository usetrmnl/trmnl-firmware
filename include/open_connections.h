#pragma once

#include <HTTPClient.h>

// While kept, one connection per host (the API and the image host) stays open between requests.
namespace OpenConnections {
  void keep(bool on); // off closes the open connections
  void closeAll(); // frees their TLS buffers; the next request opens new ones
  // An HTTPClient begun on url over the open connection to its host, or nullptr when not kept.
  HTTPClient *begin(const String &url);
  void end(HTTPClient *http, bool request_succeeded); // a failed request may leave unread bytes, so it closes
} // namespace OpenConnections
