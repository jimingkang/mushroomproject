import { DashboardHeader } from "@/components/dashboard-header"
import { VideoFeed } from "@/components/video-feed"
import { LogViewer } from "@/components/log-viewer"
import { RemoteControl } from "@/components/remote-control"
import { StatusPanel } from "@/components/status-panel"

export default function RobotMonitoringDashboard() {
  return (
    <div className="min-h-screen bg-background">
      <DashboardHeader />

      <main className="p-4 lg:p-6">
        <div className="grid gap-4 lg:gap-6 grid-cols-1 lg:grid-cols-12">
          {/* Left Column - Video Feeds */}
          <div className="lg:col-span-8 space-y-4 lg:space-y-6">
      <img src='http://172.23.248.37:5002/robot_image' />

            {/* Secondary Video Feeds */}
            <div className="grid grid-cols-1 md:grid-cols-2 gap-4 lg:gap-6">
              <VideoFeed
                cameraId="CAM-002"
                name="Top Camera"
                isActive={true}
              streamUrl="http://172.23.248.37:5002/video_feed"
              />
              <VideoFeed
                cameraId="CAM-003"
                name="Side Camera"
                isActive={true}
                streamUrl="http://172.23.248.37:5002/video_feed2"
              />
            </div>

            {/* Log Viewer */}
            <div className="h-[350px]">
              <LogViewer />
            </div>
          </div>

          {/* Right Column - Controls & Status */}
          <div className="lg:col-span-4 space-y-4 lg:space-y-6">
            <StatusPanel />
            <RemoteControl />
          </div>
        </div>
      </main>
    </div>
  )
}
