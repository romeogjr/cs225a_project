/**
 * Shared constants module for sai-interfaces.
 * @module ./config
 */

export const EVENT_CONTROLLER_STATUS = 'controller-status';
export const EVENT_RESET_DISPLAYS = 'reset-displays';
export const EVENT_RESET_ACTIVE_TABS_FROM_REDIS = 'reset-tabs-from-redis';
/**
 * Redis key for the controller status.
 * @constant
 * @type {string}
 */
export const REDIS_KEY_CONTROLLER_STATE = 'sai::examples::control_state';

/**
 * The value of REDIS_KEY_CONTROLLER_STATE in Redis when the controller is
 * ready to take commands.
 * @type {string}
 */
export const REDIS_VAL_CONTROLLER_READY = 'ready';

/**
 * The current controller primitive, e.g. posori or joint control.
 * @constant
 * @type {string}
 */
export const REDIS_KEY_CURRENT_PRIMITIVE = 'sai::examples::primitive';

export const REDIS_PRIMITIVE_UPDATING = 'sai::examples::primitive_updating';