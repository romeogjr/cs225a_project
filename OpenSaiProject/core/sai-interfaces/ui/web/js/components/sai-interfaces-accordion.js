/**
 * @module ./module/sai-interface-accordion
 */

import { get_redis_val, post_redis_key_val } from '../redis.js';

const template = document.createElement('template');
template.innerHTML = `
<style>
	.sai-interfaces-accordion-btn {
		color: white;
		cursor: pointer;
		padding: 18px;
		width: 100%;
		border: none;
		text-align: left;
		outline: none;
		transition: 0.4s;
	}

	.sai-interfaces-content {
		padding: 0 18px;
		background-color: white;
		max-height: 0;
		overflow: hidden;
		transition: max-height 0.2s ease-out;
	}
</style>
<button class="sai-interfaces-accordion-btn">
</button>
`;

customElements.define('sai-interfaces-accordion', class extends HTMLElement {
	constructor() {
		super();
		this.template = template;
		this.active = false;
	}

	connectedCallback() {
		let template_node = this.template.content.cloneNode(true);

		let toggleKey = this.getAttribute('key');
		let displayName = this.getAttribute('displayName') || toggleKey;

		let button = template_node.querySelector('.sai-interfaces-accordion-btn');

		// direct children that is not the button & styling
		let direct_children = this.querySelectorAll(':scope > :not(.sai-interfaces-accordion-btn), :not(style)');

		button.innerHTML = displayName;
		button.addEventListener('click', () => {
			post_redis_key_val(toggleKey, this.active ? 0 : 1);
			this.active = !this.active;

			if (this.active) {
				button.classList.remove("button-disable");
				button.classList.add("button-enable");
				this.refresh();
				$(direct_children).show();
			}
			else {
				button.classList.add("button-disable");
				button.classList.remove("button-enable");
				$(direct_children).hide();
			}
		});

		get_redis_val(toggleKey).then(data => {
			this.active = !!data;
			if (this.active) {
				button.classList.remove("button-disable");
				button.classList.add("button-enable");
				$(direct_children).show();
			}
			else {
				button.classList.add("button-disable");
				button.classList.remove("button-enable");
				$(direct_children).hide();
			}
		});

		// append to document
		this.prepend(template_node);
	}

	refresh() {
		for (let child of this.children) {
			if (typeof child.refresh == 'function') {
				child.refresh();
			}
		}
	}
});