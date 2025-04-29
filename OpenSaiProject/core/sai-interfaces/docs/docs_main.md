## SAI-Interfaces Documentation

sai-interfaces provides two main tools to enables users to quickly develop controllers and simulations with the sai libraries:
1. The C++ interface library that automates the generation of controllers and simulated worlds from config files, and provides a way to interact with the controllers via a set of redis keys.
2. A web browser based user interface that provides an easy way to input desired values for the controller or siulation parameters that can be changed via redis.

Concretely, in order to make an application for simulation or control, we can follow those steps:
* Chose a robot or several robots and have a model for them as a URDF file description. Some urdf files are provided in the [Sai-Model](https://github.com/manips-sai-org/sai-model) library, and are accessible easily. See the config files in the __examples/config_files__ folder in the sai-interfaces repository for examples on how to use them.
* If you want to simulate or visualize a virtual world, make a __world.urdf__ file to define the virtual world. See the examples in __examples/world_files__ in the sai-interfaces repository (in particular the [`world_panda.urdf`](https://github.com/manips-sai-org/sai-interfaces/blob/master/examples/world/panda_world.urdf) file is commented in details as a tutorial).
* Make a __config.xml__ file to define the configuration for the simulated world and the controller. Those are custom xml files that refer to a world file for the simulation, and to robot urdf models for the controllers, where several hierarchical controllers can be defined and fully parametrized. For details on the config files, see [this page](./config_files_details.md).
* Launch an instance of the `MainRedisInterface` class with the xml config file name and the path to the config_files folder, and it will generate a webui file (in `sain2-interfaces/examples/config_files/webui_generated_file`) for potential interaction at runtime, and start the simulation and controller.

If you want to use the webui, the recommended way is to use the automatically generated file and serve it to the server. 

```
python3 ui/server.py examples/config_files/webui_generated_file/webui.html
```

It should contain all the required ways to interact with the controllers and simulated world, based on the provided config file.

For advanced users and developpers, it is also possible to make a custom webui file using standard html/css/javascript, and a number of custom html elements provided in the sai-interfaces ui folder. Go to the `docs/ui_elements_details` to see the ui advanced documentation, or follow [this link](https://github.com/manips-sai-org/sai-interfaces/blob/master/docs/ui_elements_details/ui_docs_menu.md) for an online version.
