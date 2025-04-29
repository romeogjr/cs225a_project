// Define the custom web component
const template = document.createElement('template');
template.innerHTML = `
<sai-interfaces-tabs name="motion_force_task">
	<sai-interfaces-tab-content name="Motion control">
		<sai-interfaces-motion-control/>
	</sai-interfaces-tab-content>
	<sai-interfaces-tab-content name="Force control">
		<sai-interfaces-force-control />
	</sai-interfaces-tab-content>
</sai-interfaces-tabs>
`;

class SaiInterfacesMotionForceTask extends HTMLElement {
	constructor() {
		super();
	}

	connectedCallback() {
		this.robotName = this.getAttribute('robotName');
		this.controllerName = this.getAttribute('controllerName');
		this.taskName = this.getAttribute('taskName');

		// Fetch the HTML template
		const template_node = template.content.cloneNode(true);
		let motion_control = template_node.querySelector('sai-interfaces-motion-control');
		motion_control.setAttribute('robotName', this.robotName);
		motion_control.setAttribute('controllerName', this.controllerName);
		motion_control.setAttribute('taskName', this.taskName);
		if (this.hasAttribute('minGoalPosition')) {
			motion_control.setAttribute('minGoalPosition', this.getAttribute('minGoalPosition'));
		}
		if (this.hasAttribute('maxGoalPosition')) {
			motion_control.setAttribute('maxGoalPosition', this.getAttribute('maxGoalPosition'));
		}

		let force_control = template_node.querySelector('sai-interfaces-force-control');
		force_control.setAttribute('robotName', this.robotName);
		force_control.setAttribute('controllerName', this.controllerName);
		force_control.setAttribute('taskName', this.taskName);
		if (this.hasAttribute('minDesiredForce')) {
			force_control.setAttribute('minDesiredForce', this.getAttribute('minDesiredForce'));
		}
		if (this.hasAttribute('maxDesiredForce')) {
			force_control.setAttribute('maxDesiredForce', this.getAttribute('maxDesiredForce'));
		}
		if (this.hasAttribute('minDesiredMoment')) {
			force_control.setAttribute('minDesiredMoment', this.getAttribute('minDesiredMoment'));
		}
		if (this.hasAttribute('maxDesiredMoment')) {
			force_control.setAttribute('maxDesiredMoment', this.getAttribute('maxDesiredMoment'));
		}

		if (this.getAttribute('redisPrefix')) {
			this.redisPrefix = this.getAttribute('redisPrefix');
			motion_control.setAttribute('redisPrefix', this.redisPrefix);
			force_control.setAttribute('redisPrefix', this.redisPrefix);
		}

		this.replaceWith(template_node);

	}
}

// Define the custom element
customElements.define('sai-interfaces-motion-force-task', SaiInterfacesMotionForceTask);
