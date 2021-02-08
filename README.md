# Coexistance mesh

nRF52840dk&dongle

This project demonstrates how nRF5 SDK for Mesh and nRF5 SDK examples can be used together at the same time. 

### req:
* [mesh model, client, server, provisioner](https://github.com/AgentGosdepa/rtls_mesh_3) 
Put it in your mesh directory

### build:

Use segger. Put everything in your nRF_SDK_*/examples directory

Global macros:
```
SDK_ROOT=pathto/nRF5_SDK_17.0.0_9d13099
MESH_ROOT=pathto/mesh420
```

Preprocessor definitions
```
BOARD_PCA10056 for dev kit
BOARD_PCA10059 for dongle
```
