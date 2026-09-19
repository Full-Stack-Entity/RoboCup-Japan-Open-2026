"""Point-local evidence, not room coverage. Consumes already-validated adapter results."""
import math


class ViewEvidence:
    def __init__(self):
        self.reset()

    def reset(self):
        self.arrival_stamp=None;self.negatives=[];self.last_now=None

    def arrived(self,stamp):
        self.reset()
        if type(stamp) is int and stamp>0:self.arrival_stamp=stamp

    def observe(self,row,result,now):
        stamp=row.get('depth_stamp_ns')
        eligible=(self.arrival_stamp is not None and result.get('state')=='observe' and
            result.get('adapter_reason')=='no_target_data_usable_but_coverage_unverified' and
            type(stamp) is int and stamp>self.arrival_stamp and math.isfinite(now))
        if not eligible:
            self.negatives=[];self.last_now=None;return
        if self.last_now is not None and (not 0<now-self.last_now<=.5 or stamp<=self.negatives[-1]):
            self.negatives=[]
        self.negatives.append(stamp);self.negatives=self.negatives[-32:];self.last_now=now

    def snapshot(self,now):
        fresh=(self.last_now is not None and math.isfinite(now) and 0<=now-self.last_now<=.5)
        return dict(schema='handyman-view-evidence-v1',arrival_verified=self.arrival_stamp is not None,
                    arrival_stamp_ns=self.arrival_stamp,negative_stamps=list(self.negatives) if fresh else [],
                    view_data_usable=bool(fresh and len(self.negatives)>=3),coverage_verified=False)
