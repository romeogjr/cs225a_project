/**
 * Defines a custom HTML element to set a Redis key with
 * a set of predefined values.
 * <br>
 * <pre>
 * Example usage:
 * <sai-interface-enum key="long_key_name" display="friendly_key_name" values="[A,B,C]">
 * </sai-interface-enum>
 * </pre>
 * @module ./module/sai-interface-enum
 */

import { get_redis_val, post_redis_key_val } from '../redis.js';

const template = document.createElement('template');
template.innerHTML = `
<div class="form-floating">
	<select class="form-select border border-primary"></select>
	<label></label>
</div>
`;

customElements.define('sai-interfaces-enum', class extends HTMLElement {
	constructor() {
		super();
		this.template = template;
		this.display = this.getAttribute('display');
		this.key = this.getAttribute('key');
		this.values = JSON.parse(this.getAttribute('values') || '[]');

		// assumption: values are strings/numbers
		this.value_map = {};

		this.get_redis_val_and_update = this.get_redis_val_and_update.bind(this);
	}

	connectedCallback() {
		let template_node = this.template.content.cloneNode(true);
		this.label_node = template_node.querySelector('label');
		this.label_node.setAttribute('for', this.key);
		this.selector_dom = template_node.querySelector('select');
		this.selector_dom.id = this.key;

		this.label_node.innerHTML = this.display || this.key;

		this.values.forEach(option => {
			let option_node = document.createElement('option');
			option_node.value = option;
			option_node.innerHTML = option;
			this.selector_dom.appendChild(option_node);
			this.value_map[option] = option;
		});

		this.selector_dom.onchange = e => {
			let raw_option = e.target.value;

			// attempt to parse as number
			let option = parseFloat(raw_option);
			if (isNaN(option))
				option = raw_option;

			post_redis_key_val(this.key, option);
		}

		// fetch initial value from redis
		this.get_redis_val_and_update();

		// append to document
		this.appendChild(template_node);
	}

	/**
	 * Fetches the latest value of the key attribute from 
	 * Redis, and updates the internal value of this enum.
	 */
	get_redis_val_and_update() {
		get_redis_val(this.key).then(option => {
			if (!(option in this.value_map)) {
				this.selector_dom.value = this.values[0];
				post_redis_key_val(this.key, this.values[0])
			} else {
				this.selector_dom.value = option;
			}
		});
	}
});
