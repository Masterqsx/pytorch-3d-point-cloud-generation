python findLR_stg2.py --model ORIG_STG2 --experiment adam_trueWD \
	--chunkSize 100 --batchSize 32 \
	--loadPath ORIG_STG1_adam_trueWD \
	--optim adam --trueWD 1e-4 \
	--startLR 1e-7 --endLR 10 --itersLR 50 \
	--gpu 0

python findLR_stg2.py --model ORIG_STG2 --experiment sgd_trueWD \
	--chunkSize 100 --batchSize 32 \
	--loadPath ORIG_STG1_sgd_trueWD \
	--optim sgd --trueWD 1e-4 \
	--startLR 1e-7 --endLR 10 --itersLR 50 \
	--gpu 0