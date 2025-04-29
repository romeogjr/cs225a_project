import { get_redis_val, post_redis_key_val } from '../redis.js';

// Define the template for the custom element
const template = document.createElement('template');
template.innerHTML = `
<style>
.input-group input[type="number"]::-webkit-inner-spin-button {
	-webkit-appearance: none;
	margin: 0;
}

.input-group input[type="number"] {
	-moz-appearance: textfield;
}

.number_input_axis_setter {
	height: 3.4em;
}

.invisible-button {
	visibility: hidden;
}

.label-bg-gray {
	background-color: rgb(245, 245, 245);
}


</style>

<div>
    <h5></h5>
    <div class="row">
        <div class="input-group p-1">
            <span class="input-group-text">
                <p>current</p>
            </span>
            <span class="form-control label-bg-gray" aria-label="x_label"></span>
            <span class="form-control label-bg-gray" aria-label="y_label"></span>
            <span class="form-control label-bg-gray" aria-label="z_label"></span>
            <button type="button" class="btn btn-primary invisible-button">Update</button>
        </div>
    </div>
    <div class="row">
        <div class="input-group p-1">
            <span class="input-group-text">
                <p>desired</p>
            </span>
            <input type="number" aria-label="x" class="form-control number_input_axis_setter" value="0">
            <input type="number" aria-label="y" class="form-control number_input_axis_setter" value="0">
            <input type="number" aria-label="z" class="form-control number_input_axis_setter" value="1">
            <button type="button" id="update_btn" class="btn btn-primary">Update</button>
        </div>
    </div>
</div>
`;

// Define the custom element class
class SaiInterfacesAxisSetter extends HTMLElement {
	constructor() {
		super();

		this.key = this.getAttribute('key');

		// throw error if key is not provided
		if (!this.key) {
			throw new Error('key attribute must be provided');
		}

		const title = this.getAttribute('display') || this.key;
		this.appendChild(template.content.cloneNode(true));
		this.querySelector('h5').textContent = title;

		get_redis_val(this.key).then(keyval => {
			let len = (Array.isArray(keyval)) ? keyval.length : 1;
			if (len == 3) {
				const x = keyval[0];
				const y = keyval[1];
				const z = keyval[2];

				this.updateDisplays(x, y, z);
			}
		});

		// Attach click event listener to the update button
		const updateButton = this.querySelector('#update_btn');
		updateButton.id = 'update_btn_' + this.key;
		updateButton.addEventListener('click', this.handleUpdateButtonClick.bind(this));

		// attach event listener to the enter key press for the inputs
		const inputElements = this.querySelectorAll('input');
		inputElements.forEach(inputElement => {
			inputElement.addEventListener('keyup', (event) => {
				if (event.key === 'Enter') {
					this.handleUpdateButtonClick();
				}
			});
		});
	}

	handleUpdateButtonClick() {
		// Get the input elements for X, Y, and Z
		const xInput = this.querySelector('input[aria-label="x"]');
		const yInput = this.querySelector('input[aria-label="y"]');
		const zInput = this.querySelector('input[aria-label="z"]');

		// Get the values of X, Y, and Z inputs
		const xValue = parseFloat(xInput.value);
		const yValue = parseFloat(yInput.value);
		const zValue = parseFloat(zInput.value);

		// Normalize the vector
		const length = Math.sqrt(xValue ** 2 + yValue ** 2 + zValue ** 2);
		if (length < 1e-6) {
			xInput.value = xLabel.textContent;
			yInput.value = yLabel.textContent;
			zInput.value = zLabel.textContent;
			return;
		}

		const normalizedX = xValue / length;
		const normalizedY = yValue / length;
		const normalizedZ = zValue / length;

		post_redis_key_val(this.key, [normalizedX, normalizedY, normalizedZ]);

		this.updateDisplays(normalizedX, normalizedY, normalizedZ);
	}

	updateDisplays(x, y, z) {
		const xInput = this.querySelector('input[aria-label="x"]');
		const yInput = this.querySelector('input[aria-label="y"]');
		const zInput = this.querySelector('input[aria-label="z"]');

		const xLabel = this.querySelector('span[aria-label="x_label"]');
		const yLabel = this.querySelector('span[aria-label="y_label"]');
		const zLabel = this.querySelector('span[aria-label="z_label"]');

		const formattedX = x.toFixed(3).replace(/\.?0+$/, '');
		const formattedY = y.toFixed(3).replace(/\.?0+$/, '');
		const formattedZ = z.toFixed(3).replace(/\.?0+$/, '');

		// Update the input values with normalized values
		xInput.value = formattedX;
		yInput.value = formattedY;
		zInput.value = formattedZ;

		// Update the axis labels
		xLabel.textContent = formattedX;
		yLabel.textContent = formattedY;
		zLabel.textContent = formattedZ;
	}
}

// Define the custom element
customElements.define('sai-interfaces-axis-setter', SaiInterfacesAxisSetter);
