import { EVENT_CONTROLLER_STATUS } from '../config.js';


export default class SaiInterfacesComponent extends HTMLElement {
	constructor(template) {
		super();
		this.template = template;
		this.enabled = false;
		this.template_node = undefined;
	}

	connectedCallback() {
		this.template_node = this.template.content.cloneNode(true);
		this.controller_status_event = document.addEventListener(EVENT_CONTROLLER_STATUS, event => {
			this.enabled = event.detail && (event.detail.ready === true);
			if (this.enabled)
				this.enableComponent();
			else
				this.disableComponent();
		});

		this.onMount();
		this.append(this.template_node);
	}

	disconnectedCallback() {
		document.removeEventListener(EVENT_CONTROLLER_STATUS, this.controller_status_event);
		this.onUnmount();
	}

	refresh() {
	}

	enableComponent() {
		throw new Error("Not implemented!");
	}

	disableComponent() {
		throw new Error("Not implemented!");
	}

	onMount() {
		throw new Error("Not implemented!");
	}

	onUnmount() {
		throw new Error("Not implemented!");
	}

}