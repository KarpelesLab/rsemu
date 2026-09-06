-- The fixture database, written on the host by the same sqlite the guest runs.
--
-- It exists so that the guest opens a *real* SQLite file: a header it checks,
-- pages it reads at absolute offsets with pread(2), and a lock it takes with
-- fcntl(2) before it reads any of them. An in-memory database exercises none
-- of that, which is why this file is here rather than another `:memory:`.
create table city(name text, pop integer);
insert into city values ('kyoto', 1463723), ('osaka', 2691185), ('nara', 354630);
create index bypop on city(pop);
