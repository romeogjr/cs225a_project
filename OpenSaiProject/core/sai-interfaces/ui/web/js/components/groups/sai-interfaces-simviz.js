const valid_types = ['robot', 'object']

class SaiInterfacesSimviz extends HTMLElement {
	constructor() {
		super();
		this.redis_prefix = this.getAttribute('redisPrefix') || 'sai::interfaces';
	}

	connectedCallback() {

		const model_names = JSON.parse(this.getAttribute('modelNames'));
		const model_types = JSON.parse(this.getAttribute('modelTypes'));

		const model_display_names = model_names.map(name => name.replace(/_/g, ' ').replace(/\b\w/g, c => c.toUpperCase()));

		if (!this.checkAttributesValidity(model_names, model_types)) {
			const errorMessage = document.createElement('div');
			errorMessage.textContent = 'Attributes of sai-interfaces-simviz are not valid';
			errorMessage.style.color = 'red';
			errorMessage.style.fontWeight = 'bold';
			errorMessage.style.fontSize = 'larger';
			this.appendChild(errorMessage);
			return
		}

		let htmlString = `<sai-interfaces-tabs tabsOnTheLeft color="#cc7a00" name="simviz_interface">`;

		for (let i = 0; i < model_names.length; i++) {
			let model_tab_content = `<sai-interfaces-tab-content name="${model_display_names[i]}" value="${model_names[i]}">`;

			let model_ui_type = 'sai-interfaces-simviz-' + model_types[i];
			let task_ui_element = `<${model_ui_type} ${model_types[i]}Name="${model_names[i]}" redisPrefix="${this.redis_prefix}" />`;
			model_tab_content += task_ui_element;
			model_tab_content += `</sai-interfaces-tab-content>`;
			htmlString += model_tab_content;
		}

		htmlString += `
		<sai-interfaces-tab-inline-content>
			<div class="row mt-3 p-2">
				<sai-interfaces-toggle key="${this.redis_prefix}::simviz::gravity_comp_enabled" display="Gravity Compensation"/>
			</div>
			<div class="row mt-3 p-2">
				<sai-interfaces-toggle key="${this.redis_prefix}::simviz::logging_on" display="SimViz Logging"/>
			</div>
		</sai-interfaces-tab-inline-content>`;
		htmlString += `</sai-interfaces-tabs>`;

		this.innerHTML = htmlString;
	}

	checkAttributesValidity(model_names, model_types) {
		if (model_names.length == 0) {
			console.error("Model names are empty in custom html element sai-interfaces-simviz");
			return false;
		}
		if (model_names.length != model_types.length) {
			console.error("Model names and types do not have the same length in custom html element sai-interfaces-simviz");
			return false;
		}
		for (let i = 0; i < model_types.length; i++) {
			if (!valid_types.includes(model_types[i])) {
				console.error("Invalid model type in custom html element sai-interfaces-simviz");
				return false;
			}
		}
		return true;
	}
}

// Define the custom element
customElements.define('sai-interfaces-simviz', SaiInterfacesSimviz);
