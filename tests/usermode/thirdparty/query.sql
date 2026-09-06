-- Read by the guest through sqlite's own `.read`, so the shell opens a second
-- staged file while a database is already open on another descriptor.
select name from city order by pop desc;
select sum(pop) from city;
with recursive c(x) as (select 1 union all select x + 1 from c where x < 20)
  select sum(x * x) from c;
