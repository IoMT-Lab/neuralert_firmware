Sometimes a definition internal to the SDK needs to be overwritten, such as the maximum MQTT username length.
In that case, create a new header in this directory that matches the name/path of the file
you are trying to override from the SDK.

This directory is included before the SDK include directories, so it takes priority.
NOTE: making modifications this was should be avoided if at all possible. It can mess up intellisense
because editors will often think the original SDK file is the one that will be used. This can lead to 
developer confusion, as the definitions the editor thinks are being used and the definitions that are 
actually being used by the compiler will be different.