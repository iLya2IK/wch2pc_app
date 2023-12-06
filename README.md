# HTTP2 Web Camera Proto Client Application Component
API wrapper over HTTP2 REST Web Camera client as a component for the esp-idf/adf SDK. The client is designed to work in conjunction with the [REST Web Camera server](https://github.com/iLya2IK/wcwebcamserver).

# Sub-protocol description
Data exchange between devices is carried out according to the HTTP/2 protocol using the POST method. The contents of requests and responses are JSON objects. The description for JSON requests/respones inside sub-protocol you can found [here](https://github.com/iLya2IK/wcwebcamserver/wiki).
