import { post_redis_key_val, get_redis_val } from '../redis.js';

customElements.define('sai-interfaces-simple-data-structure-setter', class extends HTMLElement {
	constructor() {
		super();
		this.data = {};
	}

	connectedCallback() {
		this.key = this.getAttribute('key');

		get_redis_val(this.key).then(value => {
			this.data = value;
			this.createForm();
		});
	}

	createForm() {
		this.innerHTML = '';  // Clear any existing content

		Object.keys(this.data).forEach(field => {
			const fieldValue = this.data[field];

			// Create a container for the field
			const container = document.createElement('div');
			container.className = 'row my-2';

			// Create a label for the field
			const labelCol = document.createElement('div');
			labelCol.className = 'col-auto mx-3';
			const label = document.createElement('label');
			label.textContent = this.formatLabel(field);
			labelCol.appendChild(label);
			container.appendChild(labelCol);

			const inputCol = document.createElement('div');
			inputCol.className = 'col-auto mx-3';

			let inputElement;

			if (typeof fieldValue === 'boolean') {
				// Create a button for boolean values
				inputElement = document.createElement('button');
				inputElement.className = fieldValue ? 'btn btn-success' : 'btn btn-danger';
				inputElement.textContent = fieldValue ? 'On' : 'Off';
				inputElement.onclick = () => {
					this.data[field] = !this.data[field];
					inputElement.className = this.data[field] ? 'btn btn-success' : 'btn btn-danger';
					inputElement.textContent = this.data[field] ? 'On' : 'Off';
					this.updateRedis();
				};
			} else if (typeof fieldValue === 'number') {
				// Create an input of type number for numerical values
				inputElement = document.createElement('input');
				inputElement.type = 'number';
				inputElement.style.maxWidth = '100px';
				inputElement.className = 'form-control';
				inputElement.value = fieldValue;
				const updateValue = () => {
					const newValue = parseFloat(inputElement.value);
					if (!isNaN(newValue)) {
						this.data[field] = newValue;
						this.updateRedis();
					} else {
						inputElement.value = this.data[field];  // Revert to previous value
					}
				};
				inputElement.addEventListener('blur', updateValue);
				inputElement.addEventListener('keypress', (e) => {
					if (e.key === 'Enter') {
						updateValue();
					}
				});
			} else if (typeof fieldValue === 'string') {
				// Create an input of type text for string values
				inputElement = document.createElement('input');
				inputElement.type = 'text';
				inputElement.className = 'form-control';
				inputElement.value = fieldValue;
				const updateValue = () => {
					this.data[field] = inputElement.value;
					this.updateRedis();
				};
				inputElement.addEventListener('blur', updateValue);
				inputElement.addEventListener('keypress', (e) => {
					if (e.key === 'Enter') {
						updateValue();
					}
				});
			}

			if (inputElement) {
				inputCol.appendChild(inputElement);
			}
			container.appendChild(inputCol);

			this.appendChild(container);
		});
	}

	updateRedis() {
		post_redis_key_val(this.key, JSON.stringify(this.data))
			.catch(error => console.error('Error posting to Redis:', error));
	}

	formatLabel(label) {
		return label.split('_').map(word => word.charAt(0).toUpperCase() + word.slice(1)).join(' ');
	}

});