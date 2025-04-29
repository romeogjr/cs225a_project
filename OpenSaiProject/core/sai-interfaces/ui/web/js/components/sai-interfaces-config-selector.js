/**
 * Defines a custom HTML element for config file selection and simulation reset
 * 
 * <br>
 * Example usage:
 * &lt;sai-interfaces-logger config_key="redis-config-file-key" reset_key="redis-sim-reset-key" path_prefix="path/from/simviz/to/config/folder" /&gt;
 * HTML attributes:
 *    config_key: string - redis key in which to store the path to config file
 *    reset_key: string - redis key for the reset function
 *    path_prefix: string - prefix path of the config file (typically the folder where the config files are with respect to the sai simulation)
 * <br>
 * Note: there are no available attributes to set.
 * 
 * @module ./module/sai-interfaces-logger
 */


import { post_redis_key_val, wait_for_redis_val } from '../redis.js';
import SaiInterfacesComponent from './sai-interfaces-component.js';


const template = document.createElement('template');
template.innerHTML = `
<div class="row">
	<div class="col-9">
		<input type=file class="file_selector" placeholder="Select a config file"/>
	</div>
	<div class="col-3">
		<button class="btn btn-warning">Reset</button>
	</div>
</div>
`;

class SaiInterfacesConfigSelector extends SaiInterfacesComponent {
	constructor() {
		super(template);
		this.config_key = this.getAttribute('config_key') || "sai::interfaces::simviz::config_file";
		this.reset_key = this.getAttribute('reset_key') || "sai::interfaces::simviz::reset";
		this.path_prefix = this.getAttribute('path_prefix') || '';
	}

	onMount() {
		let config_file_input = this.template_node.querySelector('.file_selector');
		let reset_button = this.template_node.querySelector('.btn');


		// offline plotting initialization
		reset_button.onclick = async () => {
			let selectedFile = config_file_input.files[0];
			if (selectedFile) {
				let fileName = selectedFile.name; // Get the file name
				post_redis_key_val(this.config_key, this.path_prefix + fileName);
			}
			post_redis_key_val(this.reset_key, "1");
			// wait for a full reset cycle and then refresh the page
			await new Promise(r => setTimeout(r, 100));
			await wait_for_redis_val(this.reset_key, "0");
			location.reload();
		};
	}
}

customElements.define('sai-interfaces-config-selector', SaiInterfacesConfigSelector);