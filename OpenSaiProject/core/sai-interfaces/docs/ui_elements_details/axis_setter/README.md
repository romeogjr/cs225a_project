The `sai-interfaces-axis-setter` Element
=========================================
The `sai-interfaces-axis-setter` element enables the setting of a unit axis (3D vector with unit norm)

![](./init.png)

The top row shows the value in the redis server (if the key does not exist yet it will be blank), the bottom row allows the setting of the axis by setting the 3 components of the vector. When the 3 components are set, a press of the update button (or a press of the enter key when in one of the 3 inputs) will perform the following operations:
* Normalize the vector (if it is [0, 0, 0], nothing will happen and the vector will not be set)
* Set the normalized vector to redis
* Update the grayed display with the new vector value

## Usage

```
<sai-interfaces-axis-setter key="...">
</sai-interfaces-axis-setter>
```

## Attributes

* `key`: Required. The Redis key that has a rotation matrix. If the Redis key
does not exist, the element will not show.
* `display`: Optional. The displayed name for the component. If omitted, it will be the key.

## Example

We provide an [example HTML file](./axis_setter.html) containing the following html code:

```
<sai-interfaces-axis-setter key="sai::interfaces::tutorial::unit_axis" display="3D Axis" />
```

Run the server:

```
~/sai/core/sai-interfaces$ python3 ui/server.py docs/ui_elements_details/axis_setter/axis_setter.html 
```

Open a browser and go to `localhost:8000` .
Press the update button and you should see the axis update to the desired value

![](./first_set.png)

Change the desired values to something different

![](./pre-change.png)

Then press the update button to normalize the vector and set it to the redis database

![](./post-change.png)

The axis will be set to the key `sai::interfaces::tutorial::unit_axis` that can be checked in the redis database with:

```
~$ redis-cli
127.0.0.1:6379> get sai::interfaces::tutorial::unit_axis
```
