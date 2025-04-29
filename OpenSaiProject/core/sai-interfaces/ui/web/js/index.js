import {
	EVENT_CONTROLLER_STATUS,
	REDIS_VAL_CONTROLLER_READY
} from './config.js';

// Import all ES6 modules here, so the HTML template only needs to load index.js.
import './redis.js';
import './components/sai-interfaces-accordion.js';
import './components/sai-interfaces-axis-setter.js';
import './components/sai-interfaces-component.js';
import './components/sai-interfaces-config-selector.js';
import './components/sai-interfaces-display.js';
import './components/sai-interfaces-enum.js';
import './components/sai-interfaces-orientation.js';
import './components/sai-interfaces-plot-button.js';
import './components/sai-interfaces-plot.js';
import './components/sai-interfaces-setkey.js';
import './components/sai-interfaces-simple-data-structure-setter.js';
import './components/sai-interfaces-slider.js';
import './components/sai-interfaces-tabs.js';   // container
import './components/sai-interfaces-toggle-group.js';
import './components/sai-interfaces-toggle.js';

import './components/groups/sai-interfaces-force-control.js';   // single tab template
import './components/groups/sai-interfaces-joint-task.js';      // single tab template
import './components/groups/sai-interfaces-motion-control.js';  // single tab template
import './components/groups/sai-interfaces-motion-force-task.js';    // multi tab template
import './components/groups/sai-interfaces-robot-controller.js';	  // multi tab template
import './components/groups/sai-interfaces-simviz-object.js';    // single tab template
import './components/groups/sai-interfaces-simviz-robot.js';     // single tab template
import './components/groups/sai-interfaces-simviz.js';  	   // multi tab templatesla

var socket = io();
socket.on('connect', () => {
	// TODO: 
});

socket.on('controller-state', data => {
	let controller_status = data === REDIS_VAL_CONTROLLER_READY;
	document.dispatchEvent(new CustomEvent(EVENT_CONTROLLER_STATUS, { ready: controller_status }));
})

socket.on('disconnect', () => {
	// TODO: disable components
	document.dispatchEvent(new CustomEvent(EVENT_CONTROLLER_STATUS, { ready: false }));

	// attempt to reconnect
	socket.open();
})

export function throttle(func, limit) {
	let inThrottle;
	return function () {
		const context = this;
		const args = arguments;
		if (!inThrottle) {
			func.apply(context, args);
			inThrottle = true;
			setTimeout(() => inThrottle = false, limit);
		}
	};
}

export function parse_maybe_array_attribute(attr) {
	let parsed_attr;
	try {
		parsed_attr = JSON.parse(attr);
		if (parsed_attr.length == 1) {
			parsed_attr = parsed_attr[0];
		} else {
			for (let i = 0; i < parsed_attr.length; i++) {
				parsed_attr[i] = parseFloat(parsed_attr[i]);
			}
		}
	} catch (e) {
		parsed_attr = parseFloat(attr);
	}
	return parsed_attr;
}