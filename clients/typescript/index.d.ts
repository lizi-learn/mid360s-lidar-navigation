export type State = 'dispatching'|'accepted'|'running'|'blocked'|'paused'|'presenting'|'waiting'|'dwelling'|'succeeded'|'failed'|'cancelled'|'interrupted'|'unknown';
export type Target = {marker:string; x?:never; y?:never; theta?:never} | {x:number; y:number; theta?:number|null; marker?:never};
export type RelativeAction = {kind:'forward';meters:number} | {kind:'rotate';radians:number};
export interface Task {id:string; state:State; request:Record<string,unknown>; native_ref:Record<string,unknown>|null; backend:string; created_at:number; updated_at:number; observation_stale?:boolean; observation?:Record<string,unknown>; error?:{code:string;message:string;details?:Record<string,unknown>}}
export interface Marker {id?:string; name:string; x?:number; y?:number; theta?:number|null; map_id?:string|null; script?:string; dwell?:number; guest_wait?:boolean; kind?:'visit'|'transit'|'home'}
export interface Capabilities {backend:string; api_version:string; hardware:boolean; features:Record<string,boolean>; verification:string; units?:Record<string,string>; task_correlation?:string}
export interface RobotStatus {backend:string; robot_id?:string|null; map_id?:string|null; pose?:{x:number;y:number;theta:number;frame?:string}|null; navigation_state:string; navigation_permitted?:boolean|null; battery_percent:number|null; charging:boolean|null; hardware_estop:boolean|null; unknown_fields:string[]; [key:string]:unknown}
export class NavigationError extends Error {code:string;status:number;details:Record<string,unknown>;constructor(code:string,message:string,status?:number,details?:Record<string,unknown>)}
export class NavigationClient {
 constructor(baseURL?:string, options?:{apiKey?:string;timeout?:number});
 capabilities():Promise<Capabilities>;status():Promise<RobotStatus>;maps():Promise<{id:string;active:boolean}[]>;
 markers():Promise<Marker[]>;routes():Promise<Record<string,unknown>[]>;
 move(target:Target, options?:{idempotencyKey?:string}):Promise<Task>;
 visit(marker:string,options?:{idempotencyKey?:string}):Promise<Task>;
 route(routeId:string,options?:{idempotencyKey?:string}):Promise<Task>;
 forward(meters:number,options?:{idempotencyKey?:string}):Promise<Task>;
 rotate(radians:number,options?:{idempotencyKey?:string}):Promise<Task>;
 tour(markers:string[],options?:{idempotencyKey?:string}):Promise<Task>;
 sequence(steps:RelativeAction[],options?:{idempotencyKey?:string}):Promise<Task>;
 task(id:string):Promise<Task>;cancel(id:string):Promise<Task>;pause(id:string):Promise<Task>;
 resume(id:string):Promise<Task>;continueVisit(id:string):Promise<Task>;stop():Promise<{stop_requested:boolean;scope:string}>;
 saveMarker(marker:Marker):Promise<Marker>;deleteMarker(name:string):Promise<{deleted:boolean;name:string}>;
 wait(id:string,options?:{timeout?:number;interval?:number;signal?:AbortSignal}):Promise<Task>;
}
