
import { get_redis_val, post_redis_key_val } from '../redis.js';

const template = document.createElement('template');
template.innerHTML = `
<style>
	.sai-interfaces-toggle-group-top {
		display: flex;
		flex-direction: column;
		flex-wrap: wrap;
	}
</style>
<div class="sai-interfaces-toggle-group-top">
	<label>
		<input type="checkbox">
		<span class="checkable"></span>
	</label>
</div>
`;

class SaiInterfacesToggleGroup extends HTMLElement {
	constructor() {
		super();
		this.template = template;
	}

	connectedCallback() {
		let template_node = this.template.content.cloneNode(true);

		this.key = this.getAttribute("key");
		this.display = this.getAttribute("name");
		this.enabled = this.hasAttribute("enabled");

		this.container = template_node.querySelector(".sai-interfaces-toggle-group-container");
		this.checkbox = template_node.querySelector("input");
		this.checkbox.checked = this.enabled;
		this.label = template_node.querySelector("span");
		this.label.innerHTML = this.display;

		if (this.key) {
			get_redis_val(this.key).then(value => {
				this.enabled = value;
				this.checkbox.checked = value;
				this.updateGroups();
			});
		}

		this.checkbox.onchange = async e => {
			this.enabled = e.target.checked;

			this.updateGroups();

			// sleep for 100ms to allow the interface to update
			await new Promise(r => setTimeout(r, 100));

			// push update to redis if not in memory
			if (this.key) {
				post_redis_key_val(this.key, this.enabled ? 1 : 0);
			}
		};

		if (!this.key) {
			setTimeout(() => this.updateGroups(), 100);
		}

		this.prepend(template_node);
	}

	updateGroups() {
		let enabled_group = this.querySelectorAll(':scope > sai-interfaces-toggle-group-enabled');
		let disabled_group = this.querySelectorAll(':scope > sai-interfaces-toggle-group-disabled');

		enabled_group = $(enabled_group);
		disabled_group = $(disabled_group);

		if (this.enabled) {
			disabled_group.hide();
			enabled_group.each(function () {
				for (let child of this.children) {
					if (typeof child.refresh == 'function') {
						child.refresh();
					}
				}
			});
			enabled_group.show();
		} else {
			enabled_group.hide();
			disabled_group.each(function () {
				for (let child of this.children) {
					if (typeof child.refresh == 'function') {
						child.refresh();
					}
				}
			});
			disabled_group.show();
		}
	}

	refresh() {
		for (let child of this.children) {
			if (typeof child.refresh == 'function') {
				child.refresh();
			}
		}
	}
}

class ToggleGroupChildEnabled extends HTMLElement {
	refresh() {
		for (let child of this.children) {
			if (typeof child.refresh == 'function') {
				child.refresh();
			}
		}
	}
}

class ToggleGroupChildDisabled extends HTMLElement {
	refresh() {
		for (let child of this.children) {
			if (typeof child.refresh == 'function') {
				child.refresh();
			}
		}
	}
}

customElements.define('sai-interfaces-toggle-group', SaiInterfacesToggleGroup);
customElements.define('sai-interfaces-toggle-group-enabled', ToggleGroupChildEnabled);
customElements.define('sai-interfaces-toggle-group-disabled', ToggleGroupChildDisabled);