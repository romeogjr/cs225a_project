/**
 * Defines a custom HTML element that has a slider and manual value input
 * for either a scalar or vector Redis key.
 * 
 * <pre>
 * HTML tag: &lt;sai-interface-slider /&gt;
 * Attributes:
 *   * key: string - Redis key to make a slider. Must be a scalar or vector, numeric.
 *   * max: number|number[] - If scalar and key is a vector, max is applied to all elements.
 *       If both max and key are vectors, lengths must match, and each element is matched
 *       by index with a max. 
 *   * min: number|number[] - Same as max, just with min values this time.
 *   * display: string|string[]|null - Friendly display for the keys. If string and
 *       key is a vector, the index in brackets (e.g. [1]) will be appended. If display
 *       is a string[] and key is a vector, then each element will be matched by index.
 *  </pre>
 * @module ./module/sai-interface-slider
 */

import { EVENT_RESET_DISPLAYS } from '../config.js';
import { get_redis_val, post_redis_key_val } from '../redis.js';
import { parse_maybe_array_attribute, throttle } from '../index.js';
import SaiInterfacesComponent from './sai-interfaces-component.js';


const template = document.createElement('template');
template.innerHTML = `
<style>.sai-interface-slider-top {
	display: flex;
	flex-direction: column;
	flex-wrap: wrap;
	align-items: center;
}

.sai-interface-slider-top div {
	width: 100%;
}

.sai-interface-slider-top div div {
	display: flex;
	justify-content: space-between;
	align-items: center;
	width: 100%;
}

.sai-interface-slider-top div div p {
	flex: 1;
}

.sai-interface-slider-top div div input {
	width: 50%;
}

</style>

<div class="sai-interface-slider-top"/>
`;

class SaiInterfacesSlider extends SaiInterfacesComponent {
	constructor() {
		super(template);

		this.creating_sliders = false;

		document.addEventListener(EVENT_RESET_DISPLAYS, () => {
			// console.log('Reset sliders event caught!', event.detail.message);
			this.refresh();
		});
	}

	create_sliders(len) {
		if (this.creating_sliders) {
			console.log('Already creating sliders, returning');
			return;
		}
		this.creating_sliders = true;

		while (this.firstChild) {
			this.removeChild(this.firstChild);
		}
		this.template_node = this.template.content.cloneNode(true);
		this.container = this.template_node.querySelector('div');

		// generate appropriate number of sliders
		for (let i = 0; i < len; i++) {
			/* 
			 * The following js should be equivalent of this:
			 * <div>
			 *   <div>
			 * 	   <label>item name</label>
			 * 	   <input type="number" class="number">
			 *   </div>
			 *   <input type="range" class="slider">
			 * <div>
			 */

			let min = Array.isArray(this.min) ? this.min[i] : this.min;
			let max = Array.isArray(this.max) ? this.max[i] : this.max;
			let step = Array.isArray(this.step) ? this.step[i] : this.step;
			let value = Array.isArray(this.value) ? this.value[i] : this.value;

			if (isNaN(min)) {
				min = -1;
			}
			if (isNaN(max)) {
				max = 1;
			}
			if (isNaN(step) || (max-min)/step < 1000.0) {
				step = (max-min)/1000.0
			}
			if (isNaN(value)) {
				value = (min + max) / 2;
			}
			value = Math.min(max, Math.max(min, value));

			let slider_div = document.createElement('div');
			let slider_value_div = document.createElement('div');
			let slider_display = document.createElement('label');
			let slider_value_input = document.createElement('input');
			let slider = document.createElement('input');

			// set up slider name
			if (Array.isArray(this.value)) {
				if (Array.isArray(this.display)) {
					slider_display.innerHTML = this.display[i];
				} else {
					slider_display.innerHTML = (this.display || this.key) + "[" + i + "]";
				}
			} else {
				slider_display.innerHTML = this.display || this.key;
			}

			// set up manual value input for this slider
			slider_value_input.type = 'number';
			slider_value_input.className = 'value';
			slider_value_input.min = min;
			slider_value_input.max = max;
			slider_value_input.step = step;
			slider_value_input.value = value;

			// also clamp the this.value in case it was out of bounds
			if (Array.isArray(this.value))
				this.value[i] = parseFloat(slider_value_input.value);
			else
				this.value = parseFloat(slider_value_input.value);

			// set up typing event
			let sliding_value_input_callback = () => {
				let slider_val = parseFloat(slider_value_input.value);
				if (slider_val < slider_value_input.min)
					slider_val = slider_value_input.min;
				if (slider_val > slider_value_input.max)
					slider_val = slider_value_input.max;

				// HTML min/max coerces back to string, unfortunately
				slider_val = parseFloat(slider_val);
				slider_value_input.value = slider_val;
				slider.value = slider_val;

				if (Array.isArray(this.value))
					this.value[i] = slider_val;
				else
					this.value = slider_val;

				if (this.key) {
					post_redis_key_val(this.key, this.value);
				}

				if (this.onvaluechange) {
					this.onvaluechange(this.value);
				}
			}

			// Event listener for blur event (when input box loses focus)
			slider_value_input.addEventListener('blur', function () {
				sliding_value_input_callback();
			});

			// Event listener for keypress event (to detect when Enter key is pressed)
			slider_value_input.addEventListener('keypress', function (event) {
				if (event.key === "Enter") {
					sliding_value_input_callback();
				}
			});

			//   // set up mousewheel event for manual input
			//   slider_value_input.addEventListener('wheel', e => {
			//     e.preventDefault();
			//     let offset = (e.deltaY > 0 ? -1 : 1) * slider_value_input.step;
			//     let val = parseFloat(slider_value_input.value);
			//     slider_value_input.value = (val + offset).toFixed(3);
			//     sliding_value_input_callback();
			//   });

			// set up drag slider
			slider.type = 'range';
			slider.className = 'slider';
			slider.min = min;
			slider.max = max;
			slider.step = step;
			slider.value = value;

			let slider_move_callback = () => {
				let slider_val = parseFloat(slider.value);
				if (Array.isArray(this.value))
					this.value[i] = slider_val;
				else
					this.value = slider_val;

				slider_value_input.value = slider_val;

				if (this.key) {
					post_redis_key_val(this.key, this.value);
				}

				if (this.onvaluechange) {
					this.onvaluechange(this.value);
				}
			}
			// slider.oninput = throttle(slider_move_callback, 10);
			slider.oninput = slider_move_callback;

			//   slider.addEventListener('wheel', e => {
			//     e.preventDefault();
			//     let offset = (e.deltaY > 0 ? -1 : 1) * slider.step;
			//     let val = parseFloat(slider.value);
			//     slider.value = (val + offset).toFixed(3);
			//     slider.oninput();
			//   });

			// append label + manual value input to slider_value_div
			slider_value_div.appendChild(slider_display);
			slider_value_div.appendChild(slider_value_input);

			// add them all together
			slider_div.append(slider_value_div);
			slider_div.append(slider);
			this.container.append(slider_div);
		}

		// post to redis in case the values were clamped
		if (this.key) {
			post_redis_key_val(this.key, this.value);
		}

		this.appendChild(this.template_node);

		this.creating_sliders = false;
	}

	onMount() {
		this.key = this.getAttribute('key');
		this.min = this.getAttribute('min');
		this.max = this.getAttribute('max');
		this.step = this.getAttribute('step');
		this.size = this.getAttribute('size');

		if (this.hasAttribute('key') && this.hasAttribute('size')) {
			alert('Error: cannot set slider key and size. Set key if connected to redis, else size.')
		}

		// if we can parse as a JSON array, attempt to do so
		let raw_disp = this.getAttribute('display');
		try {
			this.display = JSON.parse(raw_disp);
		} catch (e) {
			this.display = raw_disp;
		}

		this.min = parse_maybe_array_attribute(this.min);
		this.max = parse_maybe_array_attribute(this.max);
		this.step = parse_maybe_array_attribute(this.step);

		// create sliders
		if (this.key) {
			get_redis_val(this.key).then(value => {
				// determine iteration bounds: 1 if scalar key, array size if vector
				let len = (Array.isArray(value)) ? value.length : 1;

				// save value
				this.value = value;

				this.create_sliders(len);
			});
		} else {
			if (this.size == 1) {
				this.value = (this.min + this.max) / 2;
				this.create_sliders(1);
			} else {
				this.value = [];
				for (let i = 0; i < this.size; i++) {
					let actual_min = Array.isArray(this.min) ? this.min[i] : this.min;
					let actual_max = Array.isArray(this.max) ? this.max[i] : this.max;
					this.value.push((actual_min + actual_max) / 2);
				}

				this.create_sliders(this.size);
			}
		}
	}

	refresh() {
		this.onMount();
	}

	onUnmount() {
	}

	enableComponent() {
	}

	disableComponent() {
	}
}



customElements.define('sai-interfaces-slider', SaiInterfacesSlider);