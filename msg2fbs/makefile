# make sure to escape newlines
MSG_TYPES=\
	amrl_msgs/RobofleetStatus\
	sensor_msgs/NavSatFix\
	nav_msgs/Odometry\
# this line intentionally left blank

FLATC_FLAGS=--gen-object-api --no-fb-import

.SUFFIXES: # disable default rules

.PHONY: all
all: update_schemas

.PHONY: clean
clean:
	-rm -f *_generated.*
	-rm -f schema.fbs

.PHONY: update_schemas
update_schemas: schema_generated.h schema_generated.ts
	cp schema_generated.h ../robofleet_client/include
	cp schema_generated.ts ../robofleet_server/src
	cp schema_generated.ts ../robofleet_webviz/src

.PHONY: schema.fbs # always rebuild in case message types change
schema.fbs:
	./msg2fbs.py -e -o schema.fbs $(MSG_TYPES)

schema_generated.h: schema.fbs
	flatc $(FLATC_FLAGS) -c schema.fbs

schema_generated.ts: schema.fbs
	flatc $(FLATC_FLAGS) -T schema.fbs
