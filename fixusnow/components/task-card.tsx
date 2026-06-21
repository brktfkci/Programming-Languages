import Link from "next/link";
import { MapPin, Clock, DollarSign, Users } from "lucide-react";
import { formatCurrency, formatDate } from "@/lib/utils";

interface TaskCardProps {
  task: {
    id: string;
    title: string;
    description: string;
    category: string;
    budget?: number | null;
    status: string;
    address: string;
    city: string;
    createdAt: string | Date;
    customer: { name?: string | null };
    applications: { id: string }[];
  };
}

const STATUS_COLORS: Record<string, string> = {
  OPEN: "bg-green-100 text-green-700",
  IN_PROGRESS: "bg-blue-100 text-blue-700",
  COMPLETED: "bg-gray-100 text-gray-600",
  CANCELLED: "bg-red-100 text-red-600",
};

export function TaskCard({ task }: TaskCardProps) {
  return (
    <Link href={`/tasks/${task.id}`}>
      <div className="bg-white border border-gray-200 rounded-xl p-5 hover:shadow-md hover:border-blue-300 transition-all cursor-pointer">
        <div className="flex items-start justify-between gap-2 mb-3">
          <div>
            <span className="text-xs font-medium text-blue-600 bg-blue-50 px-2 py-0.5 rounded-full">
              {task.category}
            </span>
            <span
              className={`ml-2 text-xs font-medium px-2 py-0.5 rounded-full ${STATUS_COLORS[task.status]}`}
            >
              {task.status.replace("_", " ")}
            </span>
          </div>
        </div>
        <h3 className="font-semibold text-gray-900 text-lg mb-1 line-clamp-1">{task.title}</h3>
        <p className="text-gray-500 text-sm mb-4 line-clamp-2">{task.description}</p>
        <div className="flex flex-wrap gap-3 text-sm text-gray-500">
          <span className="flex items-center gap-1">
            <MapPin className="h-4 w-4" />
            {task.city}
          </span>
          {task.budget && (
            <span className="flex items-center gap-1 text-green-600 font-medium">
              <DollarSign className="h-4 w-4" />
              {formatCurrency(task.budget)}
            </span>
          )}
          <span className="flex items-center gap-1">
            <Users className="h-4 w-4" />
            {task.applications.length} applicant{task.applications.length !== 1 ? "s" : ""}
          </span>
          <span className="flex items-center gap-1">
            <Clock className="h-4 w-4" />
            {formatDate(task.createdAt)}
          </span>
        </div>
      </div>
    </Link>
  );
}
