// Define the custom web component
class SaiInterfacesForceControl extends HTMLElement {
	constructor() {
		super();
		this.robotName = this.getAttribute('robotName');
		this.controllerName = this.getAttribute('controllerName');
		this.taskName = this.getAttribute('taskName');
		this.redisPrefix = "controllers::";
		this.min_desired_force = this.getAttribute('minDesiredForce');
		this.max_desired_force = this.getAttribute('maxDesiredForce');
		this.min_desired_moment = this.getAttribute('minDesiredMoment');
		this.max_desired_moment = this.getAttribute('maxDesiredMoment');
		if (this.getAttribute('redisPrefix')) {
			this.redisPrefix = this.getAttribute('redisPrefix') + "::controllers::";
		}
		this.redisPrefix += this.robotName + "::" + this.controllerName + "::" + this.taskName + "::";

		// Fetch the HTML template
		fetch('html/component_groups_templates/sai-interfaces-force-control.html')
			.then(response => response.text())
			.then(template => {
				let replacedHTML = template.replaceAll('{{_prefix_}}', this.redisPrefix);

				if (this.min_desired_force) {
					replacedHTML = replacedHTML.replaceAll('{{_min_desired_force_}}', this.min_desired_force);
				} else {
					replacedHTML = replacedHTML.replaceAll('{{_min_desired_force_}}', "-50");
				}
				if (this.max_desired_force) {
					replacedHTML = replacedHTML.replaceAll('{{_max_desired_force_}}', this.max_desired_force);
				} else {
					replacedHTML = replacedHTML.replaceAll('{{_max_desired_force_}}', "50");
				}

				if (this.min_desired_moment) {
					replacedHTML = replacedHTML.replaceAll('{{_min_desired_moment_}}', this.min_desired_moment);
				} else {
					replacedHTML = replacedHTML.replaceAll('{{_min_desired_moment_}}', "-10");
				}
				if (this.max_desired_moment) {
					replacedHTML = replacedHTML.replaceAll('{{_max_desired_moment_}}', this.max_desired_moment);
				} else {
					replacedHTML = replacedHTML.replaceAll('{{_max_desired_moment_}}', "10");
				}

				this.innerHTML = replacedHTML;
			});
	}
}

// Define the custom element
customElements.define('sai-interfaces-force-control', SaiInterfacesForceControl);
