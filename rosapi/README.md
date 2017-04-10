rosapi
===============

#### Parameters

 * `~topics_glob` (string, default '')
 * `~services_glob` (string, default '')
 * `~params_glob` (string, default '')
 
 ```Note: By default the rosapi calls for details about topics, services, and parameters will return nothing. You must specify a list of allowed resources.```
 Each of the glob parameters may contain an array of one or more match patterns. Resources that match any of the specified patterns will be returned by calls to the rosapi services.

An example launch file which enables all information to be returned.

```
  <node name="rosapi" pkg="rosapi" type="rosapi_node">
     <param name="topics_glob" value="[*]" />
     <param name="services_glob" value="[*]" />
     <param name="params_glob" value="[*]" />
  </node>
```


This example launch file enables only rosout and certain camera topics
```
  <node name="rosapi" pkg="rosapi" type="rosapi_node">
     <param name="topics_glob" value="[/rosout, /camera/rgb/*]" />
  </node>
```
