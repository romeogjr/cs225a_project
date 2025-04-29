/**
 * Defines a custom HTML element to toggle a Redis key from 0 <-> 1
 * while showing ON/OFF as a UI element.
 * 
 * Example usage:
 * &lt;sai-interface-toggle key="long_key_name" display="friendly name"/&gt;
 * 
 * @module ./module/sai-interface-toggle 
 */

import { get_redis_val, post_redis_key_val } from '../redis.js';


const template = document.createElement('template');
template.innerHTML = `
<div class="row">
	<div class="col-auto" >
		<label class="display"></label>
	</div>
	<div class="col-auto">
		<button class="button"></button>
	</div>
</div>
`;


customElements.define('sai-interfaces-toggle', class extends HTMLElement {
	constructor() {
		super();
		this.template = template;
		this.current_value = 0;
	}

	connectedCallback() {
		let template_node = this.template.content.cloneNode(true);
		let display_label = template_node.querySelector('.display');
		let button = template_node.querySelector('.button');

		this.key = this.getAttribute('key');

		// set display
		display_label.innerHTML = this.getAttribute('display') || this.key;
		get_redis_val(this.key).then(value => {
			this.current_value = parseInt(value);
			button.innerHTML = this.current_value ? "ON" : "OFF";
			button.className = this.current_value ? "button-enable" : "button-disable";
		});

		// set up listeners
		// register listeners for input textbox
		button.onclick = () => {
			this.current_value = this.current_value ? 0 : 1;
			post_redis_key_val(this.key, this.current_value);
			button.innerHTML = this.current_value ? "ON" : "OFF";
			button.className = this.current_value ? "button-enable" : "button-disable";
		};

		// append to document
		this.replaceWith(template_node);
	}
});
