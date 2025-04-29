const valid_task_types = ['motion_force_task', 'joint_task']

class SaiInterfacesRobotController extends HTMLElement {
	constructor() {
		super();
		this.robot_name = this.getAttribute('robotName');
		this.redis_prefix = this.getAttribute('redisPrefix') || 'sai::interfaces';
	}

	connectedCallback() {

		const controller_names = JSON.parse(this.getAttribute('controllerNames'));
		const controller_task_types = JSON.parse(this.getAttribute('controllerTaskTypes'));
		const controller_task_names = JSON.parse(this.getAttribute('controllerTaskNames'));
		const controller_task_selection = JSON.parse(this.getAttribute('controllerTaskSelections'));
		const min_goal_positions = JSON.parse(this.getAttribute('minGoalPositions'));
		const max_goal_positions = JSON.parse(this.getAttribute('maxGoalPositions'));
		const min_desired_forces = JSON.parse(this.getAttribute('minDesiredForces'));
		const max_desired_forces = JSON.parse(this.getAttribute('maxDesiredForces'));
		const min_desired_moments = JSON.parse(this.getAttribute('minDesiredMoments'));
		const max_desired_moments = JSON.parse(this.getAttribute('maxDesiredMoments'));

		const controller_display_names = controller_names.map(name => name.replace(/_/g, ' ').replace(/\b\w/g, c => c.toUpperCase()));
		const task_display_names = controller_task_names.map(task_names => task_names.map(name => name.replace(/_/g, ' ').replace(/\b\w/g, c => c.toUpperCase())));

		if (!this.checkAttributesValidity(controller_names, controller_task_types, controller_task_names)) {
			const errorMessage = document.createElement('div');
			errorMessage.textContent = 'Attributes of sai-interfaces-robot-controller are not valid';
			errorMessage.style.color = 'red';
			errorMessage.style.fontWeight = 'bold';
			errorMessage.style.fontSize = 'larger';
			this.appendChild(errorMessage);
			return
		}

		const redis_key_prefix_controller_robot = this.redis_prefix + '::controllers::' + this.robot_name + '::';

		let htmlString = `<sai-interfaces-tabs tabsOnTheLeft color="#cc7a00" name="${this.robot_name}_controller_selection" key="${redis_key_prefix_controller_robot}active_controller_name">`;

		for (let i = 0; i < controller_names.length; i++) {
			let controller_tab_content = `<sai-interfaces-tab-content name="${controller_display_names[i]}" value="${controller_names[i]}">`;

			if (controller_task_types[i].length == 1) {
				let task_ui_type = controller_task_types[i][0] == 'motion_force_task' ? 'sai-interfaces-motion-force-task' : 'sai-interfaces-joint-task';
				let task_ui_element = `<${task_ui_type} robotName="${this.robot_name}" controllerName="${controller_names[i]}" taskName="${controller_task_names[i][0]}" redisPrefix="${this.redis_prefix}" `;
				task_ui_element += controller_task_types[i][0] == 'joint_task' ? this.getOptionalUiJointTask(controller_task_selection[i][0]) : this.getOptionalUiMotionForceTask(min_goal_positions[i][0], max_goal_positions[i][0], min_desired_forces[i][0], max_desired_forces[i][0], min_desired_moments[i][0], max_desired_moments[i][0]);
				task_ui_element += `/>`;
				controller_tab_content += task_ui_element;
			} else {
				let task_tabs = `<sai-interfaces-tabs name="${controller_display_names[i]}_task_selection" color="#730099">`;

				for (let j = 0; j < controller_task_types[i].length; j++) {
					let task_tab_content = `<sai-interfaces-tab-content name="${task_display_names[i][j]}">`;
					let task_ui_type = controller_task_types[i][j] == 'motion_force_task' ? 'sai-interfaces-motion-force-task' : 'sai-interfaces-joint-task';
					let task_ui_element = `<${task_ui_type} robotName="${this.robot_name}" controllerName="${controller_names[i]}" taskName="${controller_task_names[i][j]}" redisPrefix="${this.redis_prefix}" `;
					task_ui_element += controller_task_types[i][j] == 'joint_task' ? this.getOptionalUiJointTask(controller_task_selection[i][j]) : this.getOptionalUiMotionForceTask(min_goal_positions[i][j], max_goal_positions[i][j], min_desired_forces[i][j], max_desired_forces[i][j], min_desired_moments[i][j], max_desired_moments[i][j]);
					task_ui_element += `/>`
					task_tab_content += task_ui_element;
					task_tab_content += `</sai-interfaces-tab-content>`;
					task_tabs += task_tab_content;
				}
				task_tabs += `</sai-interfaces-tabs>`;
				controller_tab_content += task_tabs;
			}
			controller_tab_content += `</sai-interfaces-tab-content>`;
			htmlString += controller_tab_content;
		}

		htmlString += `
		<sai-interfaces-tab-inline-content>
			<div class="row mt-3 p-2">
				<sai-interfaces-toggle key="${redis_key_prefix_controller_robot}logging_on" display="Controllers Logging"/>
			</div>
		</sai-interfaces-tab-inline-content>`;
		htmlString += `</sai-interfaces-tabs>`;

		this.innerHTML = htmlString;
	}

	checkAttributesValidity(controller_names, controller_task_types, controller_task_names) {
		if (controller_names.length == 0) {
			console.error("Controller names are empty in custom html element sai-interfaces-robot-controller");
			return false;
		}

		if (controller_names.length != controller_task_types.length || controller_names.length != controller_task_names.length) {
			console.error("Controller names, task types and task names do not have the same length in custom html element sai-interfaces-robot-controller");
			return false;
		}

		for (let i = 0; i < controller_task_types.length; i++) {
			if (controller_task_names[i].length != controller_task_types[i].length) {
				console.error("Task names do not have the same length as task types in custom html element sai-interfaces-robot-controller");
				return false;
			}
			for (let j = 0; j < controller_task_types[i].length; j++) {
				if (!valid_task_types.includes(controller_task_types[i][j])) {
					console.error("Invalid task type in custom html element sai-interfaces-robot-controller");
					return false;
				}
			}
		}

		return true;
	}

	getOptionalUiJointTask(controlled_joint_indexes) {
		let optional_ui_joint_task = "";

		if (this.hasAttribute('lowerJointLimits')) {
			if (controlled_joint_indexes.length > 0) {
				let lowerJointLimits = JSON.parse(this.getAttribute('lowerJointLimits'));
				let controlled_lowerJointLimits = controlled_joint_indexes.map(index => lowerJointLimits[index]).join(',');
				optional_ui_joint_task += `lowerJointLimits="[${controlled_lowerJointLimits}]" `;
			} else {
				optional_ui_joint_task += `lowerJointLimits="${this.getAttribute('lowerJointLimits')}" `;
			}
		}
		if (this.hasAttribute('upperJointLimits')) {
			if (controlled_joint_indexes.length > 0) {
				let upperJointLimits = JSON.parse(this.getAttribute('upperJointLimits'));
				let controlled_upperJointLimits = controlled_joint_indexes.map(index => upperJointLimits[index]).join(',');
				optional_ui_joint_task += `upperJointLimits="[${controlled_upperJointLimits}]" `;
			} else {
				optional_ui_joint_task += `upperJointLimits="${this.getAttribute('upperJointLimits')}" `;
			}
		}
		if (this.hasAttribute('jointNames')) {
			if (controlled_joint_indexes.length > 0) {
				let jointNames = JSON.parse(this.getAttribute('jointNames'));
				let controlled_jointNames = controlled_joint_indexes.map(index => jointNames[index]).join('", "');
				optional_ui_joint_task += `displayNames='["${controlled_jointNames}"]' `;
			} else {
				optional_ui_joint_task += `displayNames='${this.getAttribute('jointNames')}' `;
			}
		}
		return optional_ui_joint_task;
	}

	getOptionalUiMotionForceTask(min_goal_position, max_goal_position, min_desired_force, max_desired_force, min_desired_moment, max_desired_moment) {
		let optional_ui_motion_force_task = "";

		optional_ui_motion_force_task += `minGoalPosition="[${min_goal_position}]" `;
		optional_ui_motion_force_task += `maxGoalPosition="[${max_goal_position}]" `;
		optional_ui_motion_force_task += `minDesiredForce="[${min_desired_force}]" `;
		optional_ui_motion_force_task += `maxDesiredForce="[${max_desired_force}]" `;
		optional_ui_motion_force_task += `minDesiredMoment="[${min_desired_moment}]" `;
		optional_ui_motion_force_task += `maxDesiredMoment="[${max_desired_moment}]" `;

		return optional_ui_motion_force_task;
	}

}

// Define the custom element
customElements.define('sai-interfaces-robot-controller', SaiInterfacesRobotController);
