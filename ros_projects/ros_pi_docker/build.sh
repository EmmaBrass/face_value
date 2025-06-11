docker buildx build --platform linux/arm64 --build-arg pi_number=1 -f Dockerfile -t ros_pi_docker:arm64 --load .

# TODO... where should pi_number come from?