import { notFound } from "next/navigation";
import { prisma } from "@/lib/prisma";
import { auth } from "@/lib/auth";
import { formatCurrency, formatDate } from "@/lib/utils";
import { MapPin, Calendar, DollarSign, User } from "lucide-react";
import { TaskActions } from "./task-actions";

interface Props {
  params: Promise<{ id: string }>;
}

export default async function TaskDetailPage({ params }: Props) {
  const { id } = await params;
  const session = await auth();
  const currentUser = session?.user as { id: string; role: string } | undefined;

  const task = await prisma.task.findUnique({
    where: { id },
    include: {
      customer: { select: { id: true, name: true, image: true, phone: true } },
      applications: {
        include: {
          provider: {
            select: { id: true, name: true, image: true, providerProfile: true },
          },
        },
        orderBy: { createdAt: "desc" },
      },
      payment: true,
    },
  });

  if (!task) notFound();

  const isCustomer = currentUser?.id === task.customerId;
  const isProvider = currentUser?.role === "PROVIDER";
  const myApplication = task.applications.find((a) => a.providerId === currentUser?.id);

  const STATUS_LABELS: Record<string, string> = {
    OPEN: "Open",
    IN_PROGRESS: "In Progress",
    COMPLETED: "Completed",
    CANCELLED: "Cancelled",
  };

  const STATUS_COLORS: Record<string, string> = {
    OPEN: "bg-green-100 text-green-700",
    IN_PROGRESS: "bg-blue-100 text-blue-700",
    COMPLETED: "bg-gray-100 text-gray-600",
    CANCELLED: "bg-red-100 text-red-600",
  };

  return (
    <div className="max-w-4xl mx-auto px-4 py-10">
      <div className="bg-white rounded-2xl shadow-sm border border-gray-200 p-8 mb-6">
        <div className="flex flex-wrap items-center gap-3 mb-4">
          <span className="bg-blue-50 text-blue-600 text-sm font-medium px-3 py-1 rounded-full">
            {task.category}
          </span>
          <span className={`text-sm font-medium px-3 py-1 rounded-full ${STATUS_COLORS[task.status]}`}>
            {STATUS_LABELS[task.status]}
          </span>
        </div>

        <h1 className="text-3xl font-bold text-gray-900 mb-3">{task.title}</h1>
        <p className="text-gray-600 leading-relaxed mb-6">{task.description}</p>

        <div className="grid sm:grid-cols-2 gap-4 text-sm">
          <div className="flex items-center gap-2 text-gray-600">
            <MapPin className="h-4 w-4 text-gray-400" />
            {task.address}, {task.city}
          </div>
          {task.budget && (
            <div className="flex items-center gap-2 text-green-600 font-medium">
              <DollarSign className="h-4 w-4" />
              Budget: {formatCurrency(task.budget)}
            </div>
          )}
          <div className="flex items-center gap-2 text-gray-600">
            <Calendar className="h-4 w-4 text-gray-400" />
            Posted {formatDate(task.createdAt)}
          </div>
          <div className="flex items-center gap-2 text-gray-600">
            <User className="h-4 w-4 text-gray-400" />
            {task.customer.name || "Anonymous"}
          </div>
        </div>
      </div>

      <TaskActions
        task={{
          id: task.id,
          status: task.status,
          customerId: task.customerId,
          budget: task.budget,
          payment: task.payment
            ? { status: task.payment.status }
            : null,
          applications: task.applications.map((a) => ({
            id: a.id,
            providerId: a.providerId,
            price: a.price,
            message: a.message,
            status: a.status,
            provider: {
              id: a.provider.id,
              name: a.provider.name,
              image: a.provider.image,
              rating: a.provider.providerProfile?.rating ?? null,
              jobsDone: a.provider.providerProfile?.jobsDone ?? null,
            },
          })),
        }}
        currentUser={
          currentUser
            ? { id: currentUser.id, role: currentUser.role }
            : null
        }
        isCustomer={isCustomer}
        isProvider={isProvider}
        myApplication={
          myApplication
            ? { id: myApplication.id, price: myApplication.price, status: myApplication.status }
            : null
        }
      />
    </div>
  );
}
