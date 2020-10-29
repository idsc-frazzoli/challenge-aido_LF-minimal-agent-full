
build:
	dts build_utils aido-container-build --ignore-untagged


push: build
	dts build_utils aido-container-push



submit:
	dts challenges submit


submit-bea:
	dts challenges submit --impersonate 1639 --challenge all --retire-same-label


# submit-robotarium:
# 	dts challenges submit --challenge aido2_LF_r_pri,aido2_LF_r_pub

# submit-baseline-sim:
# 	dts challenges submit --impersonate 1639 --user-label "straight" --challenge aido3-LF-sim-validation,aido3-LF-sim-testing,aido3-LFV-sim-validation,aido3-LFV-sim-testing,aido3-LFVI-sim-validation,aido3-LFVI-sim-testing
#
# submit-baseline-real-validation:
# 	dts challenges submit --impersonate 1639 --user-label "straight" --challenge aido3-LF-real-validation,aido3-LFV-real-validation

submit-baseline:
	dts challenges submit --impersonate 1639 --user-label "straight" --challenge aido3-LF-sim-validation,aido3-LF-sim-testing,aido3-LFV-sim-validation,aido3-LFV-sim-testing,aido3-LFVI-sim-validation,aido3-LFVI-sim-testing,aido3-LF-real-validation,aido3-LFV-real-validation
