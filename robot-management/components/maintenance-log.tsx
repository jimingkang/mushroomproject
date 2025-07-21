import { CalendarClock, PenToolIcon as Tool, User } from "lucide-react"

import { Table, TableBody, TableCell, TableHead, TableHeader, TableRow } from "@/components/ui/table"
import { Badge } from "@/components/ui/badge"

interface MaintenanceLogProps {
  robotId: string
}

export function MaintenanceLog({ robotId }: MaintenanceLogProps) {
  // In a real app, you would fetch maintenance logs based on the robot ID
  const logs = [
    {
      id: "M-5001",
      type: "Scheduled",
      description: "Quarterly maintenance check",
      technician: "John Smith",
      date: "2025-06-20",
      duration: "2 hours",
      parts: ["Oil filter", "Hydraulic fluid"],
    },
    {
      id: "M-4892",
      type: "Repair",
      description: "Fixed faulty sensor in arm joint",
      technician: "Maria Rodriguez",
      date: "2025-05-12",
      duration: "4 hours",
      parts: ["Proximity sensor", "Wiring harness"],
    },
    {
      id: "M-4771",
      type: "Upgrade",
      description: "Firmware update to v3.2.1",
      technician: "Alex Chen",
      date: "2025-04-30",
      duration: "1 hour",
      parts: [],
    },
    {
      id: "M-4623",
      type: "Scheduled",
      description: "Monthly inspection",
      technician: "John Smith",
      date: "2025-03-15",
      duration: "1.5 hours",
      parts: ["Lubricant"],
    },
    {
      id: "M-4512",
      type: "Repair",
      description: "Replaced worn gear in rotation mechanism",
      technician: "Sarah Johnson",
      date: "2025-02-28",
      duration: "3 hours",
      parts: ["Gear assembly", "Bearings"],
    },
  ]

  return (
    <div className="rounded-md border">
      <Table>
        <TableHeader>
          <TableRow>
            <TableHead>Log ID</TableHead>
            <TableHead>Type</TableHead>
            <TableHead>Description</TableHead>
            <TableHead>Technician</TableHead>
            <TableHead>Date</TableHead>
            <TableHead>Duration</TableHead>
            <TableHead>Parts</TableHead>
          </TableRow>
        </TableHeader>
        <TableBody>
          {logs.map((log) => (
            <TableRow key={log.id}>
              <TableCell className="font-medium">{log.id}</TableCell>
              <TableCell>
                <Badge
                  variant={log.type === "Scheduled" ? "outline" : log.type === "Repair" ? "destructive" : "default"}
                >
                  <Tool className="mr-1 h-3 w-3" />
                  {log.type}
                </Badge>
              </TableCell>
              <TableCell>{log.description}</TableCell>
              <TableCell>
                <div className="flex items-center">
                  <User className="mr-1 h-4 w-4 text-muted-foreground" />
                  {log.technician}
                </div>
              </TableCell>
              <TableCell>
                <div className="flex items-center">
                  <CalendarClock className="mr-1 h-4 w-4 text-muted-foreground" />
                  {log.date}
                </div>
              </TableCell>
              <TableCell>{log.duration}</TableCell>
              <TableCell>
                {log.parts.length > 0 ? (
                  <div className="flex flex-wrap gap-1">
                    {log.parts.map((part, index) => (
                      <Badge key={index} variant="secondary" className="text-xs">
                        {part}
                      </Badge>
                    ))}
                  </div>
                ) : (
                  "-"
                )}
              </TableCell>
            </TableRow>
          ))}
        </TableBody>
      </Table>
    </div>
  )
}
