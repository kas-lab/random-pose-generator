#!/usr/bin/env python3
"""
RCT Data Collection Entry Point.

Usage:
    # Step 1: Visualize sampling area (no ROS needed)
    python3 -m rct_collector.main --map /path/to/map.yaml --visualize-only

    # Step 2: Pre-generate poses, inspect, save to JSON
    python3 -m rct_collector.main --map /path/to/map.yaml --generate-poses 3000

    # Step 3: Visualize the pre-generated poses
    python3 -m rct_collector.main --map /path/to/map.yaml --visualize-only \
        --presampled-poses ./rct_data/presampled_poses.json

    # Step 4: Run trials using the pre-generated poses
    python3 -m rct_collector.main --map /path/to/map.yaml --trials 3000 \
        --presampled-poses ./rct_data/presampled_poses.json

    # Resume from checkpoint
    python3 -m rct_collector.main --map /path/to/map.yaml --resume --output ./rct_data \
        --presampled-poses ./rct_data/presampled_poses.json
"""

import argparse
import logging
import os
import sys

import yaml

from rct_collector.orchestrator import OrchestratorConfig, RCTOrchestrator


def setup_logging(log_level: str = "INFO", log_file: str = None):
    handlers = [logging.StreamHandler(sys.stdout)]
    if log_file:
        os.makedirs(os.path.dirname(log_file) or ".", exist_ok=True)
        handlers.append(logging.FileHandler(log_file))
    logging.basicConfig(
        level=getattr(logging, log_level.upper()),
        format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
        datefmt="%Y-%m-%d %H:%M:%S",
        handlers=handlers,
    )


def parse_args():
    p = argparse.ArgumentParser(description="RCT Data Collector for Nav2")
    p.add_argument("--map", type=str, help="Path to map YAML file")
    p.add_argument("--trials", type=int, default=3000)
    p.add_argument("--output", type=str, default="./rct_data")
    p.add_argument("--timeout", type=float, default=180.0)
    p.add_argument("--cooldown", type=float, default=3.0)
    p.add_argument("--clearance", type=float, default=0.5)
    p.add_argument("--min-distance", type=float, default=3.0)
    p.add_argument("--max-distance", type=float, default=15.0)
    p.add_argument("--config", type=str, help="YAML config file")
    p.add_argument("--resume", action="store_true")
    p.add_argument("--visualize-only", action="store_true")
    p.add_argument("--n-samples", type=int, default=20,
                   help="Number of sample pairs to show in visualization (default: 20)")
    p.add_argument("--generate-poses", type=int, default=None, metavar="N",
                   help="Pre-generate N pose pairs and save to JSON (no ROS needed)")
    p.add_argument("--presampled-poses", type=str, default=None,
                   help="Path to pre-generated poses JSON file")
    p.add_argument("--collect-risk", action="store_true")
    p.add_argument("--scan-topic", default="/scan_raw")
    p.add_argument("--odom-topic", default="/mobile_base_controller/odom")
    p.add_argument("--robot-model", default="tiago", help="Gazebo model name")
    p.add_argument("--collision-threshold", type=float, default=0.15)
    p.add_argument("--log-level", default="INFO", choices=["DEBUG", "INFO", "WARNING", "ERROR"])
    p.add_argument("--seed", type=int, default=None)
    return p.parse_args()


def main():
    args = parse_args()

    is_offline_mode = args.visualize_only or args.generate_poses is not None
    log_file = os.path.join(args.output, "rct.log") if not is_offline_mode else None
    setup_logging(args.log_level, log_file)
    logger = logging.getLogger(__name__)

    # ── Offline modes (no ROS needed) ────────────────────────────────

    if is_offline_mode:
        if not args.map:
            logger.error("--map required for --visualize-only / --generate-poses")
            sys.exit(1)

        from rct_collector.pose_sampler import PoseSampler

        sampler = PoseSampler(
            map_yaml_path=args.map,
            obstacle_clearance_m=args.clearance,
            min_goal_distance=args.min_distance,
            max_goal_distance=args.max_distance,
            seed=args.seed,
        )
        sampler.load_map()
        os.makedirs(args.output, exist_ok=True)

        # Generate poses mode
        if args.generate_poses is not None:
            poses_path = os.path.join(args.output, "presampled_poses.json")
            poses = sampler.generate_and_save(args.generate_poses, poses_path)

            # Also save a visualization with the generated poses
            vis_path = os.path.join(args.output, "presampled_poses.png")
            sampler.visualize_sampling_area(vis_path, presampled_poses=poses)
            logger.info(f"Visualization saved to {vis_path}")
            return

        # Visualize-only mode
        if args.visualize_only:
            vis_path = os.path.join(args.output, "sampling_area.png")

            if args.presampled_poses:
                # Visualize an existing poses file
                poses = PoseSampler.load_presampled(args.presampled_poses)
            else:
                # Generate poses, save them, then visualize
                poses_path = os.path.join(args.output, "presampled_poses.json")
                poses = sampler.generate_and_save(args.n_samples, poses_path)

            sampler.visualize_sampling_area(vis_path, presampled_poses=poses)
            logger.info(f"Saved visualization to {vis_path}")
            return

    # ── Online mode (ROS required) ───────────────────────────────────

    # Build config
    if args.config:
        with open(args.config) as f:
            config = OrchestratorConfig(**yaml.safe_load(f))
    else:
        config = OrchestratorConfig(
            num_trials=args.trials,
            trial_timeout_sec=args.timeout,
            cooldown_sec=args.cooldown,
            map_yaml_path=args.map or "",
            obstacle_clearance_m=args.clearance,
            min_goal_distance=args.min_distance,
            max_goal_distance=args.max_distance,
            output_dir=args.output,
            scan_topic=args.scan_topic,
            odom_topic=args.odom_topic,
            gazebo_robot_model=args.robot_model,
            collision_threshold=args.collision_threshold,
            collect_risk_features=args.collect_risk,
            presampled_poses_path=args.presampled_poses,
        )

    if not config.map_yaml_path and not args.resume:
        logger.error("--map required (or use --resume)")
        sys.exit(1)

    orchestrator = RCTOrchestrator(config)
    orchestrator.initialize()
    orchestrator.run()


if __name__ == "__main__":
    main()