#include <http_client.h>

// See lib/trmnl/include/api-client/request_headers.h for how headers are built
void applyHeaders(HTTPClient &https, const HttpHeaderList &headers)
{
  for (const auto &header : headers)
    https.addHeader(header.first, header.second);
}
