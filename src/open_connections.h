#pragma once

#include <HTTPClient.h>

// Keeps one connection per host (the API and the image host) open between requests, so a live
// check-in and its image download skip the TCP and TLS setup. Off, every request opens its own.
namespace OpenConnections {
  void keep(bool on); // off closes the open connections
  // An HTTPClient begun on url over the open connection to its host, or nullptr when not kept.
  // Pass it to end() after the request; it stays open for the next one.
  HTTPClient *begin(const String &url);
  void end(HTTPClient *http);
} // namespace OpenConnections
