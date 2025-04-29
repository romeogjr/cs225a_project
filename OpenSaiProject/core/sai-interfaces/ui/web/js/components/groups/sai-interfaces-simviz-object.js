class SaiInterfacesSimvizObject extends HTMLElement {
	constructor() {
		super();
		this.objectName = this.getAttribute('objectName');
		this.redisPrefix =  this.getAttribute('RedisPrefix') + "::" || "";

		// Fetch the HTML template
		fetch('html/component_groups_templates/sai-interfaces-simviz-object.html')
			.then(response => response.text())
			.then(template => {
				let replacedHTML = template.replaceAll('{{_prefix_}}', this.redisPrefix);
				replacedHTML = replacedHTML.replaceAll('{{_object_name_}}', this.objectName);
				this.innerHTML = replacedHTML;
			});
	}
}

// Define the custom element
customElements.define('sai-interfaces-simviz-object', SaiInterfacesSimvizObject);
