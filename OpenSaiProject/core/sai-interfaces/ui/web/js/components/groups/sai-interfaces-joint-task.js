class SaiInterfacesJointTask extends HTMLElement {
	constructor() {
		super();
		this.robotName = this.getAttribute('robotName');
		this.controllerName = this.getAttribute('controllerName');
		this.taskName = this.getAttribute('taskName');
		this.redisPrefix = "controllers::";
		this.display_names = this.getAttribute('displayNames');
		this.joint_lower_limits = this.getAttribute('lowerJointLimits');
		this.joint_upper_limits = this.getAttribute('upperJointLimits');
		if (this.getAttribute('redisPrefix')) {
			this.redisPrefix = this.getAttribute('redisPrefix') + "::controllers::";
		}
		this.redisPrefix += this.robotName + "::" + this.controllerName + "::" + this.taskName + "::";

		// Fetch the HTML template
		fetch('html/component_groups_templates/sai-interfaces-joint-task.html')
			.then(response => response.text())
			.then(template => {
				let replacedHTML = template.replaceAll('{{_prefix_}}', this.redisPrefix);

				if (this.display_names) {
					replacedHTML = replacedHTML.replaceAll('{{_display_names_}}', this.display_names);
				} else {
					replacedHTML = replacedHTML.replaceAll('{{_display_names_}}', "desired_position");
				}

				if (this.joint_lower_limits) {
					replacedHTML = replacedHTML.replaceAll('{{_joint_lower_limits_}}', this.joint_lower_limits);
				} else {
					replacedHTML = replacedHTML.replaceAll('{{_joint_lower_limits_}}', "-3.14");
				}

				if (this.joint_upper_limits) {
					replacedHTML = replacedHTML.replaceAll('{{_joint_upper_limits_}}', this.joint_upper_limits);
				} else {
					replacedHTML = replacedHTML.replaceAll('{{_joint_upper_limits_}}', "3.14");
				}

				this.innerHTML = replacedHTML;
			});
	}
}

// Define the custom element
customElements.define('sai-interfaces-joint-task', SaiInterfacesJointTask);
