/**
 * Defines a custom HTML element to set a Redis key with a specified value
 * when clicking the button.
 * 
 * Example usage:
 * <sai-interface-button key="long_key_name" value="value_to_set" label="Button Label"/>
 * 
 * @module ./module/sai-interface-button 
 */

import { post_redis_key_val } from '../redis.js';

customElements.define('sai-interfaces-setkey', class extends HTMLElement {
	constructor() {
		super();
	}

	connectedCallback() {
		const key = this.getAttribute('key');
		const valueToSet = this.getAttribute('value');
		const label = this.getAttribute('display') || "set " + key + " to " + valueToSet;

		const button = document.createElement('button');
		button.classList.add('btn', 'btn-primary');
		button.textContent = label;

		button.addEventListener('click', () => {
			post_redis_key_val(key, valueToSet);
		});

		this.appendChild(button);
	}
});