% Zero all statistics
function [] = clearstats(obj)

  obj.stats_ncall_alx = 0;
  obj.stats_time_alx = 0;
  obj.stats_ncall_aldx = 0;
  obj.stats_time_aldx = 0;
  obj.stats_ncall_alddx = 0;
  obj.stats_time_alddx = 0;
  obj.stats_time_miter_last = 0;
  obj.stats_time_miters = 0;
  obj.stats_time_total = 0;
  obj.miter = 0;
  obj.miter_last = 0;
  obj.initer = 0;
  obj.initer_last = 0;
  obj.stats_time_fact_last = 0;
  obj.stats_time_fact = 0;
  obj.lsiter = 0;
  obj.lsiter_last = 0;


