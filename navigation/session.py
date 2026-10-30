"""Navigation intent lifecycle; independent of web rendering and gait actuators.
A left switch permits a NEW explicit start, never starts/resumes a goal itself.
"""
import math
import threading

MODES={'unknown','navigation_allowed','paused','manual'}
ACTIVE={'starting','executing','replanning'}
PENDING={'planning','ready'}

class NavigationSession:
    def __init__(self,authority_timeout=.1,replan_timeout=8.,max_replans=3):
        self.lock=threading.RLock();self.authority_timeout=authority_timeout
        self.replan_timeout=replan_timeout;self.max_replans=max_replans
        self.mode='unknown';self.verified=False;self.source=0;self.sequence=0;self.source_time=0.
        self.manual_activity=False;self.operator_connected=False;self.generation=0
        self.phase='idle';self.goal=None;self.path_points=0;self.cancel_generation=0
        self.cancel_pending=False;self.message='等待保护开关状态';self.notice_sequence=0
        self.blocked_since=None;self.replans=0;self.clear_since=None;self.last_now=None;self.prepared_with_authority=False
    def _notice(self,message):
        if message!=self.message:self.notice_sequence+=1;self.message=message
    def _cancel(self,message,phase='cancelled'):
        if self.phase in ACTIVE or self.phase in PENDING:
            self.cancel_pending=True;self.cancel_generation=self.generation
        self.phase=phase;self._notice(message);self.path_points=0;self.blocked_since=None;self.clear_since=None
    def _fresh(self,now):
        return (self.verified and self.operator_connected and self.source_time>0 and
                -.005<=now-self.source_time<=self.authority_timeout)
    def operator(self,mode,verified,connected,manual_activity,source,sequence,stamp,now):
        with self.lock:
            if (mode not in MODES or type(source) is not int or type(sequence) is not int
                    or source<=0 or sequence<=0 or not math.isfinite(stamp)
                    or stamp<=0 or not -.005<=now-stamp<=self.authority_timeout):
                self.verified=False;self.mode='unknown';self._cancel('保护开关数据无效或过期，已停止','stopped');return False
            if source==self.source and (sequence<=self.sequence or stamp<self.source_time):
                self.verified=False;self.mode='unknown';self._cancel('保护开关数据重放或时间倒退，已停止','stopped');return False
            changed=self.source and source!=self.source
            self.source,self.sequence,self.source_time=source,sequence,stamp
            self.mode=mode;self.verified=verified is True;self.operator_connected=connected is True
            self.manual_activity=manual_activity is True
            if changed:self._cancel('遥控输入源已更换，请重新确认目标','stopped')
            if not self._fresh(now):self._cancel('遥控连接或档位未核验，已停止','stopped')
            elif mode=='manual':self._cancel('E 右档：已交回手动控制','manual')
            elif mode=='paused':self._cancel('E 中档：导航暂停，速度归零','paused')
            elif mode!='navigation_allowed':self._cancel('保护开关档位未知，已停止','stopped')
            elif self.manual_activity:self._cancel('摇杆介入：已退出导航，请重新确认开始','manual')
            elif self.phase in {'idle','manual','paused','stopped','cancelled'}:
                self._notice('E 左档：可以选择目标，确认开始后才会行走')
            return self._fresh(now)
    def poll(self,now):
        with self.lock:
            if not math.isfinite(now) or (self.last_now is not None and now<self.last_now):
                self.verified=False;self._cancel('本机计时异常，已停止','stopped')
            self.last_now=now
            if (self.phase in ACTIVE or self.phase in PENDING and self.prepared_with_authority) and not self._fresh(now):self._cancel('保护开关状态过期，已停止','stopped')
            if self.phase=='replanning' and self.blocked_since is not None and now-self.blocked_since>=self.replan_timeout:
                self._cancel('障碍无法绕行，速度已归零，请手动处理','blocked')
            return self.status(now)
    def prepare(self,goal,now):
        with self.lock:
            if self.cancel_pending:raise ValueError('正在确认上个目标已取消，请稍候')
            if self.phase in ACTIVE:raise ValueError('请先取消当前导航再选择新目标')
            if not all(math.isfinite(float(goal[k])) for k in ('x','y','yaw')):raise ValueError('目标坐标无效')
            self.generation+=1;self.goal={k:float(goal[k]) for k in ('x','y','yaw')}
            self.prepared_with_authority=self._fresh(now);self.phase='planning';self.path_points=0;self._notice('正在预览路径，尚未开始行走');return self.generation
    def planned(self,generation,path_points,now):
        with self.lock:
            if generation!=self.generation or self.phase!='planning':return False
            if path_points<1:self.phase='blocked';self._notice('未找到可通行路径，速度保持为零');return False
            self.phase='ready';self.path_points=path_points;self._notice('路径已预览，等待确认开始');return True
    def start(self,generation,now,execution_ready,health_ready):
        with self.lock:
            if self.cancel_pending:raise ValueError('上个导航目标取消尚未确认')
            if generation!=self.generation or self.phase!='ready':raise ValueError('请重新选择并预览目标')
            if not self._fresh(now) or self.mode!='navigation_allowed' or self.manual_activity:raise ValueError('请将 E 放到左档并松开摇杆，等待状态核验')
            if not execution_ready:raise ValueError('真实步态接管接口尚未接通，目前只可预演')
            if not health_ready:raise ValueError('地面观测、定位或传感器未就绪，速度保持为零')
            self.phase='starting';self._notice('正在建立导航控制，尚未收到执行确认');return True
    def started(self,generation,now):
        with self.lock:
            if generation!=self.generation or self.phase!='starting' or not self._fresh(now) or self.mode!='navigation_allowed':return False
            self.phase='executing';self._notice('正在导航，E 右档或摇杆可立即接管');return True
    def cancel(self):
        with self.lock:self._cancel('目标已取消，速度归零');return self.cancel_generation
    def acknowledge_cancel(self,generation):
        with self.lock:
            if self.cancel_pending and generation==self.cancel_generation:self.cancel_pending=False;return True
            return False
    def obstacle(self,now):
        with self.lock:
            if self.phase=='executing':
                self.phase='replanning';self.blocked_since=now;self.replans=0;self.clear_since=None
                self._notice('检测到障碍：已停车，正在尝试绕行')
    def reroute(self,now,new_path,clear):
        with self.lock:
            if self.phase!='replanning':return
            if now-self.blocked_since>=self.replan_timeout:
                self._cancel('障碍无法绕行，速度已归零，请手动处理','blocked');return
            if new_path:self.replans+=1
            if not clear:self.clear_since=None
            elif self.clear_since is None:self.clear_since=now
            if new_path and clear and self._fresh(now) and self.mode=='navigation_allowed' and not self.manual_activity and self.clear_since is not None and now-self.clear_since>=.75:
                self.phase='executing';self.blocked_since=None;self._notice('已找到可通行绕行路径，继续导航')
            elif self.replans>=self.max_replans and not clear:self._cancel('多次规划后障碍仍阻挡，速度已归零','blocked')
    def finished(self,generation,succeeded):
        with self.lock:
            if generation!=self.generation or self.phase not in ACTIVE:return False
            self.phase='arrived' if succeeded else 'blocked';self._notice('已到达，速度归零' if succeeded else '无法完成导航，速度已归零，请手动处理');return True
    def status(self,now):
        with self.lock:
            fresh=self._fresh(now)
            return {'phase':self.phase,'generation':self.generation,'goal':self.goal,'path_points':self.path_points,
                'operator_mode':self.mode if fresh else 'unknown','operator_verified':fresh,
                'manual_activity':self.manual_activity,'navigation_permitted':fresh and self.mode=='navigation_allowed' and not self.manual_activity,
                'force_navigation_zero':self.phase!='executing' or not fresh or self.mode!='navigation_allowed' or self.manual_activity,
                'cancel_pending':self.cancel_pending,'cancel_generation':self.cancel_generation,'notice':self.message,
                'notice_sequence':self.notice_sequence,'replan_attempts':self.replans}
