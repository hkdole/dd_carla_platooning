REQUIRED_VERSION="6"
ifeq (, $(shell which opp_run))
$(error Cannot determine OMNeT++ version. Is opp_run in your PATH?)
endif
OPP_VERSION="$(shell opp_run -v | grep Version | sed -e "s/Version: //" -e "s/,.*//" | cut -d'.' -f 1)"
ifneq ("$(OPP_VERSION)","$(REQUIRED_VERSION)")
$(error This Makefile has been generated for OMNeT++ version $(REQUIRED_VERSION), but version $(OPP_VERSION) has been found. Please re-generate the Makefile using genmakefile.py)
endif

# tool for indexing vec files
SCAVETOOL = opp_scavetool
# results location
RESDIR = ../results
# script for merging
MERGESCRIPT = merge.R

# match all .vec files for the JoinManeuver config
JOINMANEUVER      = $(wildcard $(RESDIR)/JoinManeuver*.vec)
# change suffix from .vec to .Rdata and add the join prefix
JOINMANEUVER_DATA = $(JOINMANEUVER:$(RESDIR)/%.vec=$(RESDIR)/join.%.Rdata)

# vector index files and Rdata files
VCI = $(VECTOR:%.vec=%.vci)
RDATA = $(VECTOR:%.vec=%.Rdata)
CSV = $(VECTOR:%.vec=%.csv)

# all make targets
all: JoinManeuver.Rdata 

# to make JoinManeuver.Rdata we need to merge all files starting with join.JoinManeuver
# before this, check that all JOINMANEUVER_DATA files have been processed
$(RESDIR)/JoinManeuver.Rdata: $(JOINMANEUVER_DATA)
	$(MERGESCRIPT) $(RESDIR)/ join.JoinManeuver $(notdir $@) map-config default Rdata
JoinManeuver.Rdata: $(RESDIR)/JoinManeuver.Rdata

# to make all join.*.Rdata files we need to run the generic parser
join.%.csv: %.vec %.vci
	generic-parser.py $< map-config default join csv
join.%.Rdata: join.%.csv
	csv-to-rdata.R $<
	rm $<

# if vec files are not indexed, index them
%.vci : %.vec
	$(SCAVETOOL) index $<

# helper to print variable values. e.g.: make print-DELAY_DATA
print-%:
	@echo '$*=$($*)'

# every intermediate file is kept instead of being automatically deleted.
# .vci files are detected as intermediate and thus cancelled when the make
# command terminates. however, such files can still be needed by other
# targets, so keep them instead of re-doing indexing
.SECONDARY :
