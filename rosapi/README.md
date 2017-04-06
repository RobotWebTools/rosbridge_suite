rosapi
===============

#### Parameters

 `~topics_glob` (string, default '')
 `~services_glob` (string, default '')
 `~params_glob` (string, default '')
 
 ```Note: By default the rosapi calls for details about topics, services, and parameters will return nothing. You must specify a list of allowed resources.```
 Each of the glob parameters may contain an array of one or more match patterns. Resources that match any of the specified patterns will be returned by calls to the rosapi services.
