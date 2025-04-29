
/** 
 * Defines a custom HTML element to display the current value of a Redis key. 
 * <br>
 * <pre>
 * Element Tag: &lt;sai-interface-display&gt;
 * HTML attributes:
 *    key: string - Redis key to query
 *    refreshRate: number - How often, in seconds, to update
 *    decimalPlaces: number - How many decimal places to show
 * </pre>
 * @module ./module/sai-interfaces-display 
 */

import { get_redis_val } from '../redis.js';
import SaiInterfacesComponent from './sai-interfaces-component.js';


const template = document.createElement('template');
template.innerHTML = `
<style>
	.sai-interface-display-top {
		display: flex;
		flex-direction: row;
		flex-wrap: wrap;
		align-items: center;
		padding:
	}

	.sai-interface-display-top label {
		text-align: center;
		flex: 1;
	}

	.sai-interface-display-top table {
		border: 1px;
		border-style: solid;
		flex: 1;
	}

	.sai-interface-display-top tr:nth-child(even),
	tbody,
	tr,
	td {
		text-align: center;
		background: transparent;
	}
</style>
<div class="sai-interface-display-top">
	<label></label>
	<table></table>
</div>
`;

class SaiInterfacesDisplay extends SaiInterfacesComponent {
	constructor() {
		super(template);

		this.rows = 0;
		this.cols = 0;
	}

	/**
	 * Gets the shape (rows, cols) of a Redis value. Scalar will be (1, 1),
	 * Vector will be (n, 1), and Matrix will be (n, m).
	 * 
	 * @param {Number|Number[]|Number[][]} value 
	 * @returns (n, m) where n is the number of rows and m is number of cols
	 */
	get_shape(value) {
		let rows = 1;
		let cols = 1;
		if (Array.isArray(value)) {
			// vector or matrix: fix # rows
			rows = value.length;

			if (Array.isArray(value[0])) {
				// matrix: fix # cols
				cols = value[0].length;
			}
		}

		return [rows, cols];
	}

	/**
	 * Executed when it is time to update the displayed value.
	 */
	update_value() {
		get_redis_val(this.key).then(value => {
			this.value = value;

			[this.rows, this.cols] = this.get_shape(value);

			// default vector is column vector. swap to row if user requests
			if (this.display_row_vector && this.rows > 1 && this.cols == 1) {
				[this.rows, this.cols] = [this.cols, this.rows];
			}

			let tbl_body = document.createElement('tbody');
			for (let i = 0; i < this.rows; i++) {
				let tbl_row = document.createElement('tr');

				for (let j = 0; j < this.cols; j++) {
					let tbl_cell = document.createElement('td');
					let tbl_cell_text;

					if (this.rows == 1 && this.cols == 1) {
						// scalar
						tbl_cell_text = document.createTextNode('' + value.toFixed(this.decimalPlaces));
					} else if (this.rows > 1 && this.cols > 1) {
						// matrix
						tbl_cell_text = document.createTextNode(value[i][j].toFixed(this.decimalPlaces) + '');
					} else if (this.display_row_vector) {
						// row vector 
						tbl_cell_text = document.createTextNode(value[j].toFixed(this.decimalPlaces) + '');
					} else {
						// col vector
						tbl_cell_text = document.createTextNode(value[i].toFixed(this.decimalPlaces) + '');
					}

					tbl_cell.appendChild(tbl_cell_text);
					tbl_row.appendChild(tbl_cell);
				}

				tbl_body.appendChild(tbl_row);
			}

			this.table.innerHTML = '';
			this.table.appendChild(tbl_body);
		});
	}

	onMount() {
		this.key = this.getAttribute('key');
		this.refreshRate = this.getAttribute('refreshRate') || 1;
		this.decimalPlaces = this.getAttribute('decimalPlaces') || 3;
		this.display_text = this.getAttribute('display') || this.key;
		this.display_row_vector = this.hasAttribute('displayAsRowVector');
		this.labelOnTop = this.hasAttribute('labelOnTop');

		this.label = this.template_node.querySelector('label');
		this.table = this.template_node.querySelector('table');

		// Position the label and table based on the presence or not of the labelOnTop attribute
		if (this.labelOnTop) {
			this.container = this.template_node.querySelector('.sai-interface-display-top');
			this.container.style.flexDirection = 'column';
			this.label.style.textAlign = 'center';
			this.label.style.marginBottom = '15px';
		} else {
			this.label.style.marginRight = '5px';
		}

		this.label.innerHTML = this.display_text;
		this.update_value();

		this.poll_handle = setInterval(() => this.update_value(), this.refreshRate * 1000);
	}

	onUnmount() {
		clearInterval(this.poll_handle);
	}

	enableComponent() {
		clearInterval(this.poll_handle);
		this.poll_handle = setInterval(() => this.update_value(), this.refreshRate * 1000);
	}

	disableComponent() {
		clearInterval(this.poll_handle);
	}
}


customElements.define('sai-interfaces-display', SaiInterfacesDisplay);