/**
 * Defines a custom HTML element to plot Redis keys in real time.
 * Currently only supports plotting a scalar key as x and either
 * scalar or vector y.
 * <br>
 * Example usage: <pre>&lt;sai-interface-plot/&gt;</pre>
 * @module ./module/sai-interface-plot
 */

import { get_redis_all_keys } from '../redis.js';

const template = document.createElement('template');
template.innerHTML = `
<style>
	.sai-interface-plot-top {
		display: flex;
		flex-direction: column;
		height: 100%;
	}

	.sai-interface-plot-top .metadata {
		display: flex;
		align-items: baseline;
		flex-wrap: wrap;
		justify-content: space-around;
	}

	.sai-interface-plot-top .plot-div {
		flex: 1;
	}
</style>
<div class="sai-interface-plot-top">
	<div class="metadata">
		<select class="x_key chosen_select" data-placeholder="Select x key..." disabled>
			<option value="Time" selected>Time</option>
		</select>
		<select class="y_key chosen_select" multiple data-placeholder="Select y key..."></select>
		<h3>Rate (sec):&nbsp;</h3>
		<input class="query_rate" type="number" step="0.1">
		<h3>Window Size (sec):&nbsp;</h3>
		<input class="window_size" type="number" step="1.0">
		<button class="plot_button"></button>
		<label class="error-label" style="color:red;"><label>
	</div>
	<div class="plot-div"></div>
</div>
`;


customElements.define('sai-interfaces-plot', class extends HTMLElement {
	constructor() {
		super();
		this.template = template;

		this.plot = null;
		this.layout = {
			autosize: true,
			legend:{
				yanchor:"top",
				y:1.0,
				xanchor:"right",
				x:1.2
			},
			margin: {
				l: 50,
				r: 150,
				t: 50,
				b: 50,
			},
		};
		this.config = {
			responsive: true,
		};
		this.plotting = false;

		this.x_key = null;
		this.y_key_list = [];
		this.data = {};
		this.series = [];

		this.cntr = 1;
		this.fetchDataCallback = this.fetchDataCallback.bind(this);
	}

	/**
	 * Called when the plot needs to poll the backend for new values.
	 */
	fetchDataCallback() {
		let fetchOptions = {
			method: 'POST',
			headers: new Headers({ 'Content-Type': 'application/json' }),
			mode: 'same-origin',
			body: JSON.stringify({
				plot_id: this.plot_id
			})
		};

		fetch('/plot/data', fetchOptions)
			.then(response => response.json())
			.then(raw_response => {
				if (!raw_response.running) {
					clearTimeout(this.plotIntervalID);
				}

				return raw_response.data;
			})
			.then(data_list => {
				for (let data of data_list) {
					let x_data = data[this.x_key];
					if (!(this.x_key in this.data)) {
						this.data[this.x_key] = [];
					}
					this.data[this.x_key].push(x_data);

					for (let y_key of this.y_key_list) {
						let y_data = data[y_key];
						if (Array.isArray(y_data)) {
							for (let i = 0; i < y_data.length; i++) {
								let adj_y_key = y_key + '[' + i + ']';
								if (!(adj_y_key in this.data)) {
									this.data[adj_y_key] = [];
									this.series.push({
										x: this.data[this.x_key],
										y: this.data[adj_y_key],
										type: 'scattergl',
										name: adj_y_key.split("::").slice(-1)[0]
									});
								}

								this.data[adj_y_key].push(y_data[i]);
							}
						} else {
							if (!(y_key in this.data)) {
								this.data[y_key] = [];
								this.series.push({
									x: this.data[this.x_key],
									y: this.data[y_key],
									type: 'scattergl',
									name: y_key
								});
							}
							this.data[y_key].push(y_data);
						}
					}
				}

				let latest_time = this.data[this.x_key][this.data[this.x_key].length - 1];
				let initial_time = 0;
				if (this.window_size > 0) {
					initial_time = Math.max(latest_time - this.window_size, 0);
				}
				this.layout.xaxis = { title: 'Time', range: [initial_time, latest_time] };

				this.layout.datarevision++;
				Plotly.react(this.plot, this.series, this.layout, this.config);
			});
	}

	connectedCallback() {
		// attributes
		this.plot_type = this.getAttribute('plotType') || 'scatter';

		// get DOM elements
		let template_node = this.template.content.cloneNode(true);
		let xkey_select = template_node.querySelector('.x_key');
		let ykey_select = template_node.querySelector('.y_key');
		let query_rate_input = template_node.querySelector('.query_rate');
		query_rate_input.value = 0.1;
		let window_size_input = template_node.querySelector('.window_size');
		window_size_input.value = 30;
		this.window_size = 30.0;
		let plot_button = template_node.querySelector('.plot_button');
		let error_label = template_node.querySelector('.error-label');
		this.plot = template_node.querySelector('.plot-div');

		plot_button.innerHTML = this.plotting ? 'Stop' : 'Start';
		plot_button.className = 'button-enable';

		// populate keys list
		get_redis_all_keys().then(keys => {
			// add builtin time key
			let time_opt = document.createElement('option');
			time_opt.value = 'Time';
			time_opt.innerHTML = 'Time';
			xkey_select.append(time_opt);

			for (let key of keys.values()) {
				let opt = document.createElement('option');
				opt.value = key;
				opt.innerHTML = key;
				xkey_select.append(opt);

				let opt2 = opt.cloneNode();
				opt2.innerHTML = key;
				ykey_select.append(opt2);
			}

			$('.x_key').chosen({ width: '100%' });
			$('.y_key').chosen({ width: '100%' });

			$('.x_key').trigger("chosen:updated");
			$('.y_key').trigger("chosen:updated");
		});

		plot_button.onclick = () => {
			this.x_key = xkey_select.value;
			this.y_key_list = [];

			for (let option of ykey_select.options)
				if (option.selected)
					this.y_key_list.push(option.value);

			if (!this.x_key || this.y_key_list.length == 0) {
				error_label.innerHTML = 'Please select the x and y keys to plot.';
				return;
			}

			error_label.innerHTML = '';

			this.plotting = !this.plotting;
			if (this.plotting) {
				xkey_select.disabled = true;
				ykey_select.disabled = true;

				// reset data & series
				this.series.length = 0;
				this.data = {};
				Plotly.react(this.plot, this.series, this.layout, this.config);

				// determine rate. convert from sec -> ms
				let query_rate = parseFloat(query_rate_input.value);
				if (isNaN(query_rate) || query_rate < 0) {
					query_rate = 0.1;
					query_rate_input.value = 0.1;
				}

				this.window_size = parseFloat(window_size_input.value);
				if (isNaN(this.window_size) || this.window_size < 0) {
					this.window_size = 30.0;
					window_size_input.value = 30.0;
				}

				plot_button.innerHTML = 'Stop';
				plot_button.className = 'button-disable';
				xkey_select.disabled = false;
				ykey_select.disabled = false;

				let fetchOptions = {
					method: 'POST',
					headers: new Headers({ 'Content-Type': 'application/json' }),
					mode: 'same-origin',
					body: JSON.stringify({
						keys: [this.x_key, ...this.y_key_list],
						rate: query_rate
					})
				};

				fetch('/plot/start', fetchOptions)
					.then(response => response.json())
					.then(data => {
						this.plot_id = data['plot_id'];
						this.plotIntervalID = setInterval(this.fetchDataCallback, 100);
					});
			} else {
				let fetchOptions = {
					method: 'POST',
					headers: new Headers({ 'Content-Type': 'application/json' }),
					mode: 'same-origin',
					body: JSON.stringify({
						plot_id: this.plot_id
					})
				};

				fetch('/plot/stop', fetchOptions)
					.then(response => response.ok)
					.then(ok => {
						if (ok) {
							plot_button.innerHTML = 'Start';
							plot_button.className = 'button-enable';
							xkey_select.disabled = false;
							ykey_select.disabled = false;

							// clear plot timer callback
							clearInterval(this.plotIntervalID);
						}
					});
			}
		};

		// append to document
		this.appendChild(template_node);

		// plotly requires that we are attached to the main document root
		// so we initialize it now
		this.layout.datarevision = 1;
		Plotly.newPlot(this.plot, this.series, this.layout, this.config);
	}
});